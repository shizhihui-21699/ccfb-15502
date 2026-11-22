//! Smoke test for the `edge` binary wire protocol: feed JSON-lines on stdin,
//! expect verdict/action JSON-lines on stdout.

use std::io::Write;
use std::process::{Command, Stdio};

#[test]
fn edge_binary_emits_alarm_for_a_tailgate() {
    let exe = env!("CARGO_BIN_EXE_edge");
    let mut child = Command::new(exe)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn edge");

    {
        let stdin = child.stdin.as_mut().expect("stdin");
        // One badge for Alice, then two people cross. Keep it short: a few
        // frames of one person, then two.
        let mut t = 0i64;
        while t <= 6000 {
            // Door open 900..3600.
            let open = (900..3600).contains(&t);
            writeln!(
                stdin,
                r#"{{"t":"door","t_us":{t},"open":{open}}}"#,
                t = t * 1000
            )
            .unwrap();
            if t == 700 {
                writeln!(
                    stdin,
                    r#"{{"t":"badge","t_us":{}, "credential":"alice","decision":"granted"}}"#,
                    700_000
                )
                .unwrap();
            }
            // Radar: two people walking in from y=3 to y=-1 over 900..3400.
            let prog = ((t - 900).max(0) as f32 / 2500.0).clamp(0.0, 1.0);
            let y = 3.0 - 4.0 * prog;
            let mut pts = String::new();
            for (x, dy) in [(0.0f32, 0.0f32), (0.0, 0.7)] {
                for (dx, dz) in [(-0.05, 0.0), (0.0, 0.05), (0.05, -0.05)] {
                    if !pts.is_empty() {
                        pts.push(',');
                    }
                    pts.push_str(&format!(
                        r#"{{"x":{},"y":{},"z":{}}}"#,
                        x + dx,
                        y + dy,
                        1.45 + dz
                    ));
                }
            }
            writeln!(
                stdin,
                r#"{{"t":"radar","t_us":{t},"points":[{pts}]}}"#,
                t = t * 1000
            )
            .unwrap();
            t += 40;
        }
    }

    let out = child.wait_with_output().expect("wait");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "edge exited {:?}, stderr: {stderr}",
        out.status
    );
    assert!(
        stdout.contains("\"type\":\"verdict\""),
        "no verdict in output:\n{stdout}\nstderr: {stderr}"
    );
    // The lane-stop action must be present for a turnstile.
    assert!(stdout.contains("stop_lane"), "no lane stop:\n{stdout}");
}
