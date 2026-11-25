import http.server
import json
import re
import threading
import urllib.parse
from pathlib import Path

import pytest

from hexastorm.exposure.engine import ExposureEngine, _cli
from hexastorm.exposure.esp32_client import (
    Esp32WebClient,
    extract_calibration_facets,
    load_calibration_file,
    normalize_esp32_url,
)

_SAMPLE_SVG = """<svg xmlns="http://www.w3.org/2000/svg" width="5mm" height="4mm" viewBox="0 0 5 4">
  <rect x="1.0" y="1.0" width="2.0" height="2.0" fill="#000000" />
</svg>
"""


def _start_mock_esp32(
    password: str = "testpw", facets: dict | None = None
) -> tuple[http.server.ThreadingHTTPServer, dict]:
    store = {
        "facets": facets
        or {
            "0": {"scan": 0.005, "orth": -0.002},
            "1": {"scan": -0.004, "orth": 0.003},
            "2": {"scan": 0.001, "orth": 0.001},
            "3": {"scan": -0.002, "orth": -0.002},
        },
        "uploads": {},
    }

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, format, *args):  # noqa: A002
            return

        def _is_authed(self) -> bool:
            return "session=valid_tok" in self.headers.get("Cookie", "")

        def _send_json(self, code: int, payload: dict) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self._send_json(200, {"page": "home" if self._is_authed() else "login"})
                return
            if not self._is_authed():
                self._send_json(401, {"error": "Unauthorized"})
                return
            if self.path == "/api/settings":
                self._send_json(200, {"calibration": {"facets": store["facets"]}})
                return
            self._send_json(404, {"error": "Not found"})

        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length)
            if self.path == "/":
                parsed = urllib.parse.parse_qs(body.decode("utf-8"))
                pw = parsed.get("password", [""])[0]
                self.send_response(302)
                if pw == password:
                    self.send_header("Set-Cookie", "session=valid_tok; Path=/")
                self.send_header("Location", "/")
                self.end_headers()
                return

            if not self._is_authed():
                self._send_json(401, {"error": "Unauthorized"})
                return

            if self.path == "/upload":
                disp = self.headers.get("Content-Disposition", "")
                match = re.search(r'filename="?([^";]+)"?', disp)
                fname = Path(match.group(1)).name if match else "unnamed.pat"
                store["uploads"][fname] = body
                self._send_json(200, {"success": "upload succeeded", "file": fname})
                return

            self._send_json(404, {"error": "Not found"})

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, store


def test_extract_calibration_facets_formats_and_validation(tmp_path):
    """Verify calibration facet extraction from /api/settings, history list, and direct dict."""
    assert normalize_esp32_url("192.168.1.249/") == "http://192.168.1.249"
    assert normalize_esp32_url("https://hex.local/") == "https://hex.local"

    raw_facets = {
        "0": {"scan": 0.01, "orth": -0.02, "extra": 99},
        "1": {"scan": -0.01, "orth": 0.02},
    }
    expected = {
        "0": {"scan": 0.01, "orth": -0.02},
        "1": {"scan": -0.01, "orth": 0.02},
    }

    assert extract_calibration_facets({"calibration": {"facets": raw_facets}}) == expected
    assert extract_calibration_facets([{"facets": {}}, {"facets": raw_facets}]) == expected
    assert extract_calibration_facets(raw_facets) == expected

    cal_file = tmp_path / "esp_config.json"
    cal_file.write_text(json.dumps({"calibration": {"facets": raw_facets}}))
    assert load_calibration_file(cal_file) == expected

    with pytest.raises(ValueError, match="No facet calibration entries"):
        extract_calibration_facets({"calibration": {"facets": {}}})


def test_esp32_client_auth_env_and_upload(tmp_path, monkeypatch):
    """Verify Esp32WebClient session auth, HEXASTORM_PASSWORD env var, fetch, and upload."""
    server, store = _start_mock_esp32(password="4242")
    url = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        # Wrong password raises explicit auth failure
        bad_client = Esp32WebClient(url, password="wrong")
        with pytest.raises(RuntimeError, match="Authentication failed"):
            bad_client.fetch_calibration()

        # Numeric password coerced to str (Fire CLI parses bare digits as int)
        good_client = Esp32WebClient(url, password=4242)  # type: ignore[arg-type]
        facets = good_client.fetch_calibration()
        assert facets["0"] == {"scan": 0.005, "orth": -0.002}

        pat_path = tmp_path / "demo_e1_cor.pat"
        pat_path.write_bytes(b"HEXASTORM_TEST_PAYLOAD")
        resp = good_client.upload_file(pat_path)
        assert resp["success"] == "upload succeeded"
        assert store["uploads"]["demo_e1_cor.pat"] == b"HEXASTORM_TEST_PAYLOAD"

        # HEXASTORM_PASSWORD env fallback works when password is omitted
        monkeypatch.setenv("HEXASTORM_PASSWORD", "4242")
        env_client = Esp32WebClient(url)
        assert env_client.fetch_calibration()["3"]["orth"] == -0.002
    finally:
        server.shutdown()
        server.server_close()


def test_exposure_engine_positiveresist_and_cli_esp32_workflow(tmp_path):
    """Verify positiveresist inversion plus CLI --calibration, --esp32, --upload, and PCB flags."""
    svg_path = tmp_path / "mini_pcb.svg"
    svg_path.write_text(_SAMPLE_SVG, encoding="utf-8")

    # positiveresist toggles interval polarity in ExposureEngine
    neg_out = tmp_path / "neg.pat"
    pos_out = tmp_path / "pos.pat"
    neg_engine = ExposureEngine(correction=False, exposures=1, positiveresist=False)
    neg_engine.generate(svg_path, output_path=neg_out)
    pos_engine = ExposureEngine(correction=False, exposures=1, positiveresist=True)
    pos_engine.generate(svg_path, output_path=pos_out)
    assert neg_engine.readbin()["data"].sum() < pos_engine.readbin()["data"].sum()

    # Local JSON --calibration produces *_cor.pat
    cal_json = tmp_path / "cal.json"
    cal_json.write_text(
        json.dumps(
            {
                "calibration": {
                    "facets": {
                        "0": {"scan": 0.01, "orth": 0.0},
                        "1": {"scan": 0.0, "orth": 0.0},
                        "2": {"scan": 0.0, "orth": 0.0},
                        "3": {"scan": 0.0, "orth": 0.0},
                    }
                }
            }
        )
    )
    cor_target = tmp_path / "from_json.pat"
    _cli(
        input_file=str(svg_path),
        output_file=str(cor_target),
        calibration=str(cal_json),
        plot=False,
    )
    assert (tmp_path / "from_json_e1_cor.pat").is_file()

    # --esp32 + --upload fetches facets from mock controller, slices *_cor.pat, and uploads it
    server, store = _start_mock_esp32(password="hex")
    url = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        esp_target = tmp_path / "pcb_job.pat"
        _cli(
            input_file=str(svg_path),
            output_file=str(esp_target),
            esp32=url,
            upload=True,
            positiveresist=True,
            mirror_x=True,
            rotation=90.0,
            plot=False,
        )
        local_cor = tmp_path / "pcb_job_e1_cor.pat"
        assert local_cor.is_file()
        assert store["uploads"]["pcb_job_e1_cor.pat"] == local_cor.read_bytes()

        # --esp32 + --correction=False + --upload produces *_nocor.pat without fetching facets
        nocor_target = tmp_path / "pcb_nocor.pat"
        _cli(
            input_file=str(svg_path),
            output_file=str(nocor_target),
            esp32=url,
            correction=False,
            upload=True,
            plot=False,
        )
        local_nocor = tmp_path / "pcb_nocor_e1_nocor.pat"
        assert local_nocor.is_file()
        assert store["uploads"]["pcb_nocor_e1_nocor.pat"] == local_nocor.read_bytes()
    finally:
        server.shutdown()
        server.server_close()

    with pytest.raises(ValueError, match="--upload requires --esp32"):
        _cli(input_file=str(svg_path), upload=True, plot=False)
