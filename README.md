# door_tailgate

门禁闸口的**尾随进入检测**引擎，用 Rust 写，跑在门口边缘设备上，**完全本地实时**：无网络、无墙钟、无云端依赖。

融合四路信号——刷卡记录、门磁开关时长、红外/光幕、毫米波雷达点云——判断**每一次开门周期实际通过了几个人**，发现尾随立即告警并联动闸门/闸机。

门磁承担两件事：切分**开门周期**（判定结算的边界），以及久开/虚掩检测（`ProppedDoor`）。门开时长还作为**诊断证据**写进判定理由（"多少人用了毫秒级的开合"），帮助运维快速判断尾随特征，但判定本身以人数为准，不受慢闭门器或被人扶住的门影响。

```
雷达点云 ─► 聚类(DBSCAN) ─► 躯干拆分(并排) ─► 多目标跟踪(α-β) ─► 内线越线计数 ─┐
刷卡读卡器 ─► 去抖去重 ─────────────────────────────────────────────────────────┤
门磁 ─► 去抖 / 开门周期切分 ─────────────────────────────────────────────────────┤─► 开门周期
红外光幕 ─► 行程分类(人/车/噪声) ───────────────────────────────────────────────┘        │
                                                                                        ▼
                                                 实际通过人数(person equivalents) vs 授权数(刷卡)
                                                                                        │
                                                        判定 ─► 停道 / 声光报警 / 抓拍 / 语音
```

---

## 为什么这样建模

门禁门是**共享资源**：谁刷卡谁开门，但门扇会开着让后面好几个人通过。所以不能按"每次刷卡 = 一个人"来记账，而要按**开门周期（episode）**结算：

```
刷卡 ──► 开门 ──► 人员通过 ──► 关门 + 稳定窗口 ──► 判定
```

- **授权数 `authorized`** = 该周期内去重后的"通过"刷卡次数（开门前 3 秒内的预刷也算，关门后 1.5 秒内的补刷也算，因为同事常在门还在晃时刷卡）。
- **通过数 `passed`** = 从门外侧越过内线（`inner_line_m`）向内的人当量之和。
- **`passed > authorized` 即尾随**。

这套记账天然消化了需求里的三个难点：

### 难点一：搬设备、推推车

雷达前段把"工人推推车"识别为**一个** `Person` 单元（`cart_evidence=true`），而不是"一个人 + 一辆车两个目标"。车体本身只在**没有可见的人**时才成为 `Vehicle`，且 `Vehicle` 的人当量是 **0**（货不算人）。

因此单独一个推车工人：`passed=1`，`authorized=1`，不报警。

关键技术选择：**不追踪车**。早期版本把车和推车人当成两个耦合目标、靠"伴行时长"再折叠，结果车的预测位置常常离推车人的躯干更近，两者身份逐帧互换，把一次正常推车判成两个人。改成"以人为中心"后，这个抖动从根上消失。车的唯一痕迹是 `cart_evidence`，用来：给红外补数放行、允许短暂遮挡时轨迹续命。

如果车把后面的第二个人挡住了雷达，则用**红外人行程补数**（见下）。

### 难点二：多人并排正常通行

雷达前段对**人物高度的回波**单独做一次 DBSCAN，得到"躯干"。一个点团里有 ≥2 个躯干 → ≥2 个人。这解决了两人并肩时点云粘连的问题（并排时肩膀会把点云连成一团，但两个躯干在胸口高度仍是分开的）。

两名同事并排、各自刷卡：`authorized=2`，`passed=2`，不报警。
两人并排、只刷一张卡：`authorized=1`，`passed=2`，**报警**——这是真尾随。

### 难点三：门口人流高峰

高峰期每个人都刷卡，`authorized` 随人流增长，`passed` 同步增长，不会误报。防滥用的两道闸：

- **读卡去抖**：同一张卡对同一读卡器在 `debounce_ms`（默认 700ms）内的重复读算一次刷。
- **单周期授权上限** `max_authorized_per_episode`（默认 8）：防止一张卡被反复刷或被复制后"授权"一整支队伍。

### 立即告警，不等关门

判定不等门关。一旦 `passed > authorized`，就在**最后一次通过后 `alarm_settle_ms`（默认 700ms）**出判定（给同事补刷留出时间），但**不超过首次超额后 `alarm_max_delay_ms`（默认 2.5 秒）**。实测尾随者越线后约 1 秒内出告警并停道。

---

## 传感器融合与降级

雷达是**主要计数**传感器，因为它能区分人。红外是**冗余**通道，只在三种情况下补位，绝不喧宾夺主：

| 场景 | 雷达正常 | 雷达失明（无目标） |
|---|---|---|
| 单独工人推车 | 1（车不计数） | 红外 1 |
| 推车 + 背后一个人 | 雷达常能分出 2；否则红外人行程 +1 | 红外 2 |
| 并排两人各刷卡 | 2 | 红外宽行程按 1，判 `Suspect` |
| 人流高峰 | 每人一个 | 红外人行程逐个计 |

融合规则见 `Episode::passed`：

- 雷达**有过任何目标**（`radar_seen`）：`max(雷达数, min(红外人数, 雷达数 + beam_raise_max))`。红外只能**加**雷达漏掉的人（遮挡、拆不开），且**有上限**，所以一条抽风的红外不会凭空造出一群人。
- 雷达**一个目标都没看到**：红外数就是权威值。

**失明即降级，不是静默放行。** 健康状态机（`update_health`）在"门开着或红外被挡，但雷达 600ms 内没有任何有效目标"时判定 `RadarBlind`；此时任何超额都**至少是 `Suspect`**，且 `stop_lane_on_suspect` 默认会让闸机停转。传感器坏掉不会让尾随者悄悄溜过。

---

## 安全联锁（必须读）

- **带闭门器的铰链门/平移门，联锁永不锁死。** 火灾时人必须能出去。`interlock.lane_kind = "door"` 时引擎不会发 `StopLane`，只发声光报警（`AlarmRelay`）和抓拍请求（`RequestClip`）。
- **闸机/摆闸**（`lane_kind = "turnstile"`）可以停道（`StopLane`），这是"联动闸门"的正解。
- **失明降级**：`fail_secure_on_blind` 打开时，闸机在传感器失明期间保持锁止。仅对闸机/道闸，绝不对逃生门。
- 紧急按钮/消防信号应直接旁路本引擎（在硬件层实现），引擎不做任何阻挡。

---

## 快速开始

```bash
# 跑一遍所有内置场景，看判定
cargo run --bin sim -- --list
cargo run --bin sim -- --scenario tailgate
cargo run --bin sim -- --scenario cart_with_tailgater
cargo run --bin sim -- --scenario rush

# 全量测试（单元 + 端到端场景 + 二进制协议）
cargo test

# 作为边缘进程运行：JSON-lines 进，JSON-lines 出
cargo run --release --bin edge -- --config config.example.json < inputs.jsonl
```

### 内置场景（`sim`）

| 场景 | 期望 |
|---|---|
| `single` | 一人刷卡进入 → clear |
| `tailgate` | 一人刷卡、陌生人尾随 → ALARM + 停道 |
| `cart_alone` | 工人独自推车 → clear |
| `cart_with_tailgater` | 推车 + 背后尾随 → ALARM |
| `abreast_ok` | 两人并排、各刷卡 → clear |
| `abreast_piggyback` | 两人并排、一张卡 → ALARM |
| `rush` | 早高峰 6 人各刷卡 → clear |
| `zero_badge` | 无人刷卡进入 → ALARM |
| `radar_blind_beam` | 雷达全瞎、红外看到两人一次刷卡 → SUSPECT + 停道 |
| `one_credential_crowd` | 一张卡放 5 人 → ALARM |

---

## 集成协议（`edge` 二进制）

stdin 每行一个输入 JSON，stdout 每行一个事件 JSON。全程单线程、无网络。

**输入**（`t` 区分类型）：

```jsonc
{"t":"radar","t_us":1000000,"points":[{"x":0.0,"y":2.4,"z":1.45,"rcs":-4.0,"vr":-1.2}]}
{"t":"door","t_us":1000040,"open":true}
{"t":"badge","t_us":1000080,"credential":"cuid-8f21","reader":"door3-in","decision":"granted"}
{"t":"beam","t_us":1000120,"blocked":true,"direction":"in","sensor":0}
{"t":"tick","t_us":1000160}
```

- `t_us` 是**设备单调时钟**（微秒）。无墙钟依赖；日志落地时再挂墙钟偏移。
- `credential` 用不透明标识，不要放原始卡号。
- `beam.direction` 可选（`in`/`out`），只有双元定向光幕能给；没有就省略，引擎按无方向处理。
- `decision` 可选（`granted`/`denied`/省略=未知）。

**输出**（`type` 区分）：

```jsonc
{"type":"passage","episode":1,"passage":{"ts":2760000,"track":3,"x":0.02,"pe":1,"source":"radar","note":null}}
{"type":"verdict","episode":1,"ts":4040000,"severity":"alarm","authorized":1,"passed":2,"excess":1,"health":"nominal","reasons":["2 person(s) passed, 1 badge(s) granted"],"last_crossing_at":3320000}
{"type":"action","kind":"stop_lane","episode":1}
{"type":"action","kind":"alarm_relay","episode":1,"on":true}
{"type":"action","kind":"request_clip","episode":1,"from":0,"to":8320000}
{"type":"action","kind":"local_voiced_alert","episode":1,"text":"请勿尾随，请单独刷卡"}
{"type":"action","kind":"stop_lane","episode":1}
```

动作 → 硬件映射由边缘 BSP 负责：

| `Action` | 硬件 |
|---|---|
| `StopLane` / `ReleaseLane` | 闸机锁止 / 放行（**仅闸机，铰链门忽略**） |
| `AlarmRelay` | 干接点 → 声光报警器 / 门控蜂鸣 |
| `RequestClip` | 向 NVR 请求该时间段视频 |
| `LocalVoicedAlert` | 本地喇叭语音 |
| `ProppedDoor` | 门久开未关告警 |
| `Degraded` | 告警给运维平台（传感器需检修） |
| `Cleared` | 人工/补刷解除报警 |

**性能**：单帧处理在门口数十个点的量级上是微秒级；无堆分配进入稳态（除每帧点/单元向量，容量有界于 `tracker.max_tracks`）。确定性——同一输入流产生同一事件序列，所以录制的现场数据可作回归测试。

---

## 配置

所有参数见 `config.example.json`，每一项在 `src/config.rs` 都有注释和默认值。启动时会 `validate()`，物理上不合理的几何（内线在门外等）直接拒绝启动。

**上线调参要点：**

1. **几何先量准。** `lane.width_m`、`inner_line_m`（刚性线放在门内侧约 0.3–0.5m）、`outer_line_m`（进近线）。线放错，计数全错。
2. **雷达高度门限按现场标定。** `person_min_height_m`、`vehicle_max_height_m` 取决于雷达安装高度和点云坐标系。带推车测试：工人单独推车应当 `passed=1`。
3. **红外行程时长按现场标定。** `min_person_block_ms`（低于此人=噪声）、`cart_block_ms`（高于此人=车/队列）。拿秒表测单人快步、推车通过的实际遮挡时长。
4. **告警节奏**：门口纪律松（同事常一前一后）就调大 `alarm_settle_ms`；要更即时就调小，但要容忍多出的补刷窗口误报。
5. **先用 `sim` 和录制回放验证**，再上现场。把现场一段真实 trace 通过 `edge` 回放，人工核对每个 `episode` 的判定。

---

## 已知局限（诚实说明）

- **传感器物理分辨率是硬上限。** 两人贴得比雷达分辨率还近（约 < 0.3–0.4m），雷达分不开，此时依赖红外；若同时红外也分不清（一前一后紧贴穿过单一光幕会被算作一段长遮挡），就只能判 `Suspect` 而非高置信告警。这是物理限制，不是算法能绕过的。**现场应配双元定向光幕或多对射**来分清单人/双人。
- **未见 height 的雷达**（`height_available=false`）无法做躯干拆分，并排行人会退化为按宽度估计，可靠性下降；推车也无法靠高度区分。建议用带高度（3D/4D）毫米波雷达。
- **单光幕数人天生不可靠**：一段连续遮挡可能是 1 个慢走的人，也可能是 3 个人依次通过。引擎因此把"宽行程"当 1 个当量且判 `Suspect`，而不是猜一个数。
- **"允许同事代刷"的站点**（一人刷、多人过属正常）本引擎会报警；这属策略问题，应在门禁策略层解决，而不是在检测层放宽。
- 本仓库提供的是**检测引擎与协议**，不含具体雷达/光幕/闸机的驱动和 BSP；硬件映射见上表。

---

## 模块结构

| 文件 | 职责 |
|---|---|
| `src/time.rs` | 单调时间戳 `Ts` / 时长 `Dur`（微秒，无墙钟） |
| `src/geom.rs` | 位置、越线几何 |
| `src/config.rs` | 带默认值与校验的站点配置 |
| `src/radar.rs` | 点云 → DBSCAN 聚类 → 躯干拆分 → `Unit`（人/车 + 车证据） |
| `src/tracker.rs` | 多目标跟踪（α-β、类别门控关联）→ 内线越线 |
| `src/badge.rs` | 读卡去抖去重 |
| `src/door.rs` | 门磁去抖、开门/关门稳定沿 |
| `src/beam.rs` | 红外行程 → 噪声/人/宽（车或队列） |
| `src/engine.rs` | 开门周期记账、传感器融合、判定、联锁动作 |
| `src/bin/edge.rs` | 边缘进程：JSON-lines 协议 |
| `src/bin/sim.rs` | 场景仿真与可视化时间线 |
| `tests/scenarios.rs` | 端到端验收场景 |
| `tests/edge_protocol.rs` | 二进制协议冒烟 |
