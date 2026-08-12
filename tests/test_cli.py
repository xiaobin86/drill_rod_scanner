from drill_rod_scanner.cli import load_config


def test_load_config(tmp_path):
    cfg = tmp_path / "scanner.yaml"
    cfg.write_text(
        "servo:\n"
        "  port: /dev/ttyUSB0\n"
        "  baudrate: 115200\n"
        "lidar:\n"
        "  port: /dev/ttyUSB1\n"
        "  baudrate: 115200\n"
        "scan:\n"
        "  from_deg: 0.0\n"
        "  to_deg: 20.0\n"
        "  step_deg: 10.0\n"
        "  settle_time_s: 0.0\n"
        "stitch:\n"
        "  voxel_size: null\n"
        "output:\n"
        "  dir: out\n"
        "  cloud_format: ply\n"
    )
    cfg_obj = load_config(cfg)
    assert cfg_obj["servo"]["port"] == "/dev/ttyUSB0"
    assert cfg_obj["scan"]["from_deg"] == 0.0
