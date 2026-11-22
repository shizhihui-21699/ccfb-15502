#[test]
fn example_config_parses_and_validates() {
    let s = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/config.example.json")).unwrap();
    let cfg = door_tailgate::Config::from_json_str(&s).expect("parse");
    assert!(cfg.validate().is_empty(), "{:?}", cfg.validate());
}
