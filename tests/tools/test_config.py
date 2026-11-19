from hexastorm.config import Camera, PlatformConfig, Spi


def test_camera_specs_and_defaults():
    """Verify centralized OV2640 and OV2311 camera specifications."""
    assert Camera.OV2640["name"] == "OV2640"
    assert Camera.OV2640["width"] == 1600
    assert Camera.OV2640["height"] == 1200
    assert Camera.OV2640["pixel_size_um"] == 2.2
    assert Camera.OV2640["svga_pixel_size_um"] == 4.4
    assert Camera.OV2640["i2c_addr"] == 0x30

    assert Camera.OV2311["name"] == "OV2311"
    assert Camera.OV2311["width"] == 1600
    assert Camera.OV2311["height"] == 1300
    assert Camera.OV2311["pixel_size_um"] == 3.0
    assert Camera.OV2311["line_time_ms"] == 0.0189
    assert Camera.DEFAULT_PIXEL_SIZE_UM == 3.0


def test_platform_config_camera_cfg_and_esp32_cfg_isolation():
    """Verify PlatformConfig exposes camera_cfg and builds fresh esp32 camera dicts."""
    cfg = PlatformConfig(test=False)

    cam_cfg = cfg.camera_cfg
    assert set(cam_cfg.keys()) == {"ov2640", "ov2311"}
    assert cam_cfg["ov2640"] is Camera.OV2640
    assert cam_cfg["ov2311"] is Camera.OV2311

    esp_cfg = cfg.esp32_cfg
    esp_cam = esp_cfg["camera"]
    expected_pins = {
        "d0",
        "d1",
        "d2",
        "d3",
        "d4",
        "d5",
        "d6",
        "d7",
        "pclk",
        "vsync",
        "href",
        "xclk",
        "xclk_freq",
        "pwdn",
        "reset",
    }
    assert expected_pins.issubset(esp_cam.keys())
    for pin_name in expected_pins:
        assert esp_cam[pin_name] == Camera.OV2640["pins"][pin_name]
    assert esp_cam["settings"] == Camera.OV2640["settings"]
    assert esp_cfg["i2c"]["cam_addr"] == Camera.OV2640["i2c_addr"] == 0x30

    # Mutating one esp32_cfg['camera'] snapshot must not mutate Camera.OV2640
    esp_cam["reset"] = 999
    esp_cam["settings"]["exposure_ms"] = 1
    fresh_cam = cfg.esp32_cfg["camera"]
    assert fresh_cam["reset"] == Camera.OV2640["pins"]["reset"]
    assert (
        fresh_cam["settings"]["exposure_ms"] == Camera.OV2640["settings"]["exposure_ms"]
    )


def test_spi_words_move_staticmethod():
    """Spi.words_move works cleanly via both class and instance calls."""
    hdl_cfg = {"motors": 3, "pol_degree": 2}
    via_class = Spi.words_move(hdl_cfg)
    via_instance = Spi().words_move(hdl_cfg)
    assert via_class == via_instance
    assert isinstance(via_class, int)
    assert via_class > 0
