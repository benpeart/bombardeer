"""Tests for externalized YAML config loading and ARMED password guard rules."""

from bombardeer_pkg.config import load_config


def test_load_default_config_disables_armed(tmp_path):
    cfg = load_config(str(tmp_path / "missing.yaml"), env={})

    assert cfg.camera.frame_width == 1280
    assert cfg.camera.frame_height == 720
    assert cfg.camera.sensor_day_size == (4608, 2592)
    assert cfg.camera.sensor_dusk_size == (2304, 1296)
    assert cfg.control.fire_data_freshness_sec == 0.220
    assert cfg.control.coast_decay_onset_sec == 0.650
    assert cfg.control.coast_max_horizon_sec == 1.800
    assert cfg.optics.max_engage_range_m == 40.0
    assert cfg.control.tilt_velocity_deadband_mrad == 10.0
    assert cfg.wildlife.max_animal_height_m == 1.85
    assert not hasattr(cfg.wildlife, "veto_padding_px")
    assert cfg.web.auth_pass == cfg.web.default_pass
    assert cfg.web.armed_allowed is False


def test_default_env_password_still_disables_armed(tmp_path):
    cfg = load_config(str(tmp_path / "missing.yaml"), env={"BOMBARDEER_PASS": "bombardeer"})
    assert cfg.web.armed_allowed is False


def test_load_yaml_and_custom_env_enables_armed(tmp_path):
    yaml_file = tmp_path / "config.yaml"
    yaml_file.write_text(
        "optics:\n"
        "  tilt_calibration_trim_mrad: -12.5\n"
        "  horizon_y_offset_720: 18.0\n"
        "  max_engage_range_m: 38.0\n"
        "camera:\n"
        "  sensor_day_size: [4608, 2592]\n"
        "wildlife:\n"
        "  max_animal_height_m: 2.0\n",
        encoding="utf-8",
    )
    cfg = load_config(
        str(yaml_file),
        env={"BOMBARDEER_USER": "operator", "BOMBARDEER_PASS": "StrongYardPass!99"},
    )
    assert cfg.optics.tilt_calibration_trim_mrad == -12.5
    assert cfg.optics.horizon_y_offset_720 == 18.0
    assert cfg.optics.max_engage_range_m == 38.0
    assert cfg.camera.sensor_day_size == (4608, 2592)
    assert cfg.wildlife.max_animal_height_m == 2.0
    assert cfg.web.auth_user == "operator"
    assert cfg.web.auth_pass == "StrongYardPass!99"
    assert cfg.web.armed_allowed is True
