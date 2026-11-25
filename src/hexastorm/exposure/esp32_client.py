import getpass
import http.cookiejar
import json
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any


def normalize_esp32_url(url: str) -> str:
    """Normalize a controller URL or host/IP string into an HTTP(S) base URL."""
    cleaned = str(url).strip().rstrip("/")
    if not cleaned:
        raise ValueError("ESP32 URL must not be empty")
    if not cleaned.startswith(("http://", "https://")):
        cleaned = f"http://{cleaned}"
    return cleaned


def extract_calibration_facets(data: Any) -> dict[str, dict[str, float]]:
    """Extract ``{facet_id: {"scan": float, "orth": float}}`` from supported JSON payloads.

    Accepts:
    - ESP32 ``/api/settings`` or ``config.json`` (``{"calibration": {"facets": ...}}``)
    - PC ``calibration_history.json`` list (``[..., {"facets": ...}]``)
    - Direct ``{"facets": ...}`` or ``{"0": {"scan": ..., "orth": ...}}`` dicts
    """
    payload = data
    if isinstance(payload, list):
        if not payload:
            raise ValueError("Calibration history list is empty")
        payload = payload[-1]

    if not isinstance(payload, dict):
        raise ValueError("Calibration data must be a JSON object or non-empty list")

    if isinstance(payload.get("calibration"), dict):
        payload = payload["calibration"]
    if isinstance(payload.get("facets"), dict):
        payload = payload["facets"]

    if not isinstance(payload, dict) or not payload:
        raise ValueError("No facet calibration entries found in payload")

    facets: dict[str, dict[str, float]] = {}
    for facet_key, entry in payload.items():
        if not isinstance(entry, dict) or "scan" not in entry or "orth" not in entry:
            raise ValueError(
                f"Calibration entry {facet_key!r} is missing 'scan' or 'orth'"
            )
        facets[str(facet_key)] = {
            "scan": float(entry["scan"]),
            "orth": float(entry["orth"]),
        }
    return facets


def load_calibration_file(path: str | Path) -> dict[str, dict[str, float]]:
    """Load and normalize facet calibration offsets from a local JSON file."""
    cal_path = Path(path)
    if not cal_path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {cal_path}")
    raw = json.loads(cal_path.read_text(encoding="utf-8"))
    return extract_calibration_facets(raw)


class Esp32WebClient:
    """Minimal authenticated HTTP client for the ESP32-S3 Hexastorm webserver."""

    def __init__(
        self,
        base_url: str,
        password: str | None = None,
        timeout: float = 15.0,
    ) -> None:
        self.base_url = normalize_esp32_url(base_url)
        self._explicit_password = password is not None
        if password is not None:
            self.password = str(password)
        else:
            self.password = os.environ.get("HEXASTORM_PASSWORD", "hex")
        self.timeout = float(timeout)
        self._cookie_jar = http.cookiejar.CookieJar()
        self._opener = urllib.request.build_opener(
            urllib.request.HTTPCookieProcessor(self._cookie_jar)
        )
        self._authenticated = False
        self._cached_settings: dict[str, Any] | None = None

    def _post_login(self, password: str) -> None:
        form = urllib.parse.urlencode({"password": password}).encode("utf-8")
        req = urllib.request.Request(
            f"{self.base_url}/",
            data=form,
            method="POST",
            headers={"Content-Type": "application/x-www-form-urlencoded"},
        )
        with self._opener.open(req, timeout=self.timeout) as resp:
            resp.read()

    def _get_settings(self) -> dict[str, Any]:
        req = urllib.request.Request(f"{self.base_url}/api/settings", method="GET")
        with self._opener.open(req, timeout=self.timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def login(self, prompt_on_fail: bool = True) -> dict[str, Any]:
        """Authenticate session cookie against ``POST /`` and return ``/api/settings``."""
        self._post_login(self.password)
        try:
            settings = self._get_settings()
        except urllib.error.HTTPError as exc:
            if exc.code != 401:
                raise
            if (
                not self._explicit_password
                and prompt_on_fail
                and sys.stdin is not None
                and sys.stdin.isatty()
            ):
                entered = getpass.getpass(
                    f"ESP32 web password for {self.base_url}: "
                )
                self._post_login(entered)
                try:
                    settings = self._get_settings()
                except urllib.error.HTTPError as retry_exc:
                    if retry_exc.code == 401:
                        raise RuntimeError(
                            f"Authentication failed against {self.base_url} "
                            "(pass --password or set HEXASTORM_PASSWORD)"
                        ) from retry_exc
                    raise
                self.password = entered
            else:
                raise RuntimeError(
                    f"Authentication failed against {self.base_url} "
                    "(pass --password or set HEXASTORM_PASSWORD)"
                ) from exc

        self._authenticated = True
        self._cached_settings = settings
        return settings

    def fetch_calibration(self) -> dict[str, dict[str, float]]:
        """Fetch live ``calibration.facets`` offsets from the ESP32 controller."""
        settings = (
            self._cached_settings
            if self._authenticated and self._cached_settings is not None
            else self.login()
        )
        return extract_calibration_facets(settings)

    def upload_file(
        self, file_path: str | Path, remote_name: str | None = None
    ) -> dict[str, Any]:
        """Upload a compiled ``.pat`` or G-code file to ``POST /upload``."""
        if not self._authenticated:
            self.login()

        src_path = Path(file_path)
        payload = src_path.read_bytes()
        name = Path(remote_name).name if remote_name else src_path.name
        safe_name = name.replace('"', "_")

        req = urllib.request.Request(
            f"{self.base_url}/upload",
            data=payload,
            method="POST",
            headers={
                "Content-Type": "application/octet-stream",
                "Content-Disposition": f'attachment; filename="{safe_name}"',
                "Content-Length": str(len(payload)),
            },
        )
        try:
            with self._opener.open(req, timeout=max(self.timeout, 60.0)) as resp:
                raw = resp.read().decode("utf-8").strip()
                return json.loads(raw) if raw else {"status": resp.status}
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace").strip()
            raise RuntimeError(
                f"Upload to {self.base_url}/upload failed (HTTP {exc.code}): {detail}"
            ) from exc
