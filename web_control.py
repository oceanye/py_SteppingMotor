"""Small, dependency-free LAN web server for the motor control GUI.

The server deliberately knows nothing about Tk or the serial protocol.  The GUI
passes a controller object implementing these methods::

    web_get_status()
    web_stepper_move(axis, direction, distance_mm, speed_mm_s)
    web_stepper_stop(axis)
    web_motor_command(mode, axis, action, target_deg=None)
    web_track_command(action, pwm=0.0, lease_ms=0)
    web_emergency_stop()

Controller methods are invoked on HTTP worker threads.  A Tk based controller
must marshal GUI work to its main thread before touching widgets.
"""

from __future__ import annotations

import json
import math
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import urlsplit


MAX_REQUEST_BODY = 16 * 1024
MAX_REQUEST_TARGET = 2 * 1024
MAX_STEPPER_AXIS = 29
WEB_ROOT = Path(__file__).resolve().parent / "web"
INDEX_FILE = WEB_ROOT / "index.html"


class _ControlHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], owner: "WebControlServer"):
        self.owner = owner
        super().__init__(address, _RequestHandler)


class _RequestHandler(BaseHTTPRequestHandler):
    server: _ControlHTTPServer
    protocol_version = "HTTP/1.1"

    def log_message(self, _format: str, *_args: object) -> None:
        # Keep the embedded server quiet; operational information is shown by
        # the desktop GUI instead of being written to stderr.
        return

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = self._safe_target()
        if parsed is None:
            return
        if parsed.path in ("/", "/index.html"):
            self._serve_index()
            return
        if parsed.path == "/api/status":
            self._invoke(self.server.owner.controller.web_get_status)
            return
        self._error(HTTPStatus.NOT_FOUND, "未知路径")

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = self._safe_target()
        if parsed is None:
            return
        routes = {
            "/api/stepper/move": self._stepper_move,
            "/api/stepper/stop": self._stepper_stop,
            "/api/stepper/config": self._stepper_config,
            "/api/motor": self._motor,
            "/api/track": self._track,
            "/api/estop": self._estop,
        }
        action = routes.get(parsed.path)
        if action is None:
            self._error(HTTPStatus.NOT_FOUND, "未知路径")
            return
        body = self._read_json_object()
        if body is None:
            return
        try:
            action(body)
        except ValueError as exc:
            self._error(HTTPStatus.BAD_REQUEST, str(exc))

    def do_OPTIONS(self) -> None:  # noqa: N802
        # The bundled page is same-origin.  Refusing permissive CORS keeps other
        # LAN pages from turning a browser into an unprompted control client.
        self._error(HTTPStatus.METHOD_NOT_ALLOWED, "不支持跨域请求")

    def _safe_target(self):
        if len(self.path) > MAX_REQUEST_TARGET:
            self._error(HTTPStatus.REQUEST_URI_TOO_LONG, "请求路径过长")
            return None
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc or not parsed.path.startswith("/"):
            self._error(HTTPStatus.BAD_REQUEST, "无效请求路径")
            return None
        # Only exact paths are routed.  Explicitly reject path normalisation and
        # traversal forms before any filesystem access is attempted.
        if "\\" in parsed.path or "//" in parsed.path or any(
            part in (".", "..") for part in parsed.path.split("/")
        ):
            self._error(HTTPStatus.BAD_REQUEST, "无效请求路径")
            return None
        return parsed

    def _read_json_object(self) -> dict[str, Any] | None:
        content_type = self.headers.get_content_type().lower()
        if content_type != "application/json":
            self._error(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "仅接受 application/json")
            return None
        raw_length = self.headers.get("Content-Length")
        try:
            length = int(raw_length) if raw_length is not None else -1
        except ValueError:
            length = -1
        if length < 0:
            self._error(HTTPStatus.LENGTH_REQUIRED, "缺少有效的 Content-Length")
            return None
        if length > MAX_REQUEST_BODY:
            self._error(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "请求体过大")
            return None
        try:
            value = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._error(HTTPStatus.BAD_REQUEST, "JSON 格式错误")
            return None
        if not isinstance(value, dict):
            self._error(HTTPStatus.BAD_REQUEST, "JSON 顶层必须是对象")
            return None
        return value

    @staticmethod
    def _keys(body: Mapping[str, Any], required: set[str], optional: set[str] = set()) -> None:
        missing = required - body.keys()
        unknown = body.keys() - required - optional
        if missing:
            raise ValueError("缺少字段：" + ", ".join(sorted(missing)))
        if unknown:
            raise ValueError("未知字段：" + ", ".join(sorted(unknown)))

    @staticmethod
    def _integer(value: Any, name: str, low: int, high: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
            raise ValueError(f"{name} 必须是 {low}..{high} 的整数")
        return value

    @staticmethod
    def _number(value: Any, name: str, low: float, high: float) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} 必须是数字")
        result = float(value)
        if not math.isfinite(result) or not low <= result <= high:
            raise ValueError(f"{name} 必须在 {low:g}..{high:g} 范围内")
        return result

    def _stepper_move(self, body: dict[str, Any]) -> None:
        self._keys(body, {"axis", "direction", "distance_mm", "speed_mm_s"})
        axis = self._integer(body["axis"], "axis", 0, MAX_STEPPER_AXIS)
        direction = body["direction"]
        if direction not in ("forward", "reverse"):
            raise ValueError("direction 必须是 forward 或 reverse")
        distance = self._number(body["distance_mm"], "distance_mm", 0.001, 100000.0)
        speed = self._number(body["speed_mm_s"], "speed_mm_s", 0.001, 10000.0)
        self._invoke(
            self.server.owner.controller.web_stepper_move,
            axis,
            direction,
            distance,
            speed,
        )

    def _stepper_stop(self, body: dict[str, Any]) -> None:
        self._keys(body, {"axis"})
        axis = self._integer(body["axis"], "axis", 0, MAX_STEPPER_AXIS)
        self._invoke(self.server.owner.controller.web_stepper_stop, axis)

    def _stepper_config(self, body: dict[str, Any]) -> None:
        self._keys(body, {"axis"}, {"mode", "pulse_per_rev", "gear_ratio", "lead_mm"})
        axis = self._integer(body["axis"], "axis", 0, MAX_STEPPER_AXIS)
        kwargs: dict[str, Any] = {}
        if "mode" in body:
            m = body["mode"]
            if m not in ("linear", "rotary"):
                raise ValueError("mode 必须是 linear 或 rotary")
            kwargs["mode"] = m
        if "pulse_per_rev" in body:
            kwargs["pulse_per_rev"] = self._number(body["pulse_per_rev"], "pulse_per_rev", 1.0, 10000.0)
        if "gear_ratio" in body:
            kwargs["gear_ratio"] = self._number(body["gear_ratio"], "gear_ratio", 0.001, 1000.0)
        if "lead_mm" in body:
            kwargs["lead_mm"] = self._number(body["lead_mm"], "lead_mm", 0.01, 100.0)
        self._invoke(self.server.owner.controller.web_stepper_config, axis, **kwargs)

    def _motor(self, body: dict[str, Any]) -> None:
        self._keys(body, {"mode", "axis", "action"}, {"target_deg"})
        mode = body["mode"]
        action = body["action"]
        if mode not in ("GEAR", "FOC"):
            raise ValueError("mode 必须是 GEAR 或 FOC")
        axis = self._integer(body["axis"], "axis", 0, 1)
        if action not in ("enable", "disable", "zero", "target"):
            raise ValueError("action 必须是 enable、disable、zero 或 target")
        target = None
        if action == "target":
            if "target_deg" not in body:
                raise ValueError("target 操作缺少 target_deg")
            target = self._number(body["target_deg"], "target_deg", -3600.0, 3600.0)
        elif "target_deg" in body:
            raise ValueError("该操作不接受 target_deg")
        self._invoke(self.server.owner.controller.web_motor_command, mode, axis, action, target)

    def _track(self, body: dict[str, Any]) -> None:
        self._keys(body, {"action"}, {"pwm", "lease_ms"})
        action = body["action"]
        if action not in ("forward", "reverse", "stop"):
            raise ValueError("action 必须是 forward、reverse 或 stop")
        if action == "stop":
            if set(body) != {"action"}:
                raise ValueError("stop 操作只接受 action")
            pwm, lease_ms = 0.0, 0
        else:
            if "pwm" not in body or "lease_ms" not in body:
                raise ValueError("运行操作必须包含 pwm 和 lease_ms")
            pwm = self._number(body["pwm"], "pwm", 1.0, 100.0)
            lease_ms = self._integer(body["lease_ms"], "lease_ms", 250, 2000)
        self._invoke(self.server.owner.controller.web_track_command, action, pwm, lease_ms)

    def _estop(self, body: dict[str, Any]) -> None:
        self._keys(body, set())
        self._invoke(self.server.owner.controller.web_emergency_stop)

    def _invoke(self, method, *args: Any, **kwargs: Any) -> None:
        try:
            result = method(*args, **kwargs)
            if result is None:
                result = {"ok": True}
            elif not isinstance(result, Mapping):
                result = {"ok": True, "result": result}
            self._json(HTTPStatus.OK, result)
        except (ValueError, RuntimeError) as exc:
            self._error(HTTPStatus.CONFLICT, str(exc))
        except Exception:
            # Do not disclose serial, filesystem or implementation details to a
            # LAN client.  The GUI can log the original exception if desired.
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, "控制器执行失败")

    def _serve_index(self) -> None:
        try:
            content = INDEX_FILE.read_bytes()
        except OSError:
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, "网页文件不可用")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Connection", "close")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; "
            "img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'",
        )
        self.end_headers()
        self.close_connection = True
        self.wfile.write(content)

    def _json(self, status: HTTPStatus, value: Mapping[str, Any]) -> None:
        try:
            content = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
        except (TypeError, ValueError):
            status = HTTPStatus.INTERNAL_SERVER_ERROR
            content = b'{"ok":false,"error":"status is not JSON serializable"}'
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        # Some rejected POSTs intentionally leave an oversized or unauthorised
        # body unread.  Closing prevents those bytes being parsed as a second
        # request on the same connection.
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        self.wfile.write(content)

    def _error(self, status: HTTPStatus, message: str) -> None:
        self._json(status, {"ok": False, "error": message})


class WebControlServer:
    """Lifecycle wrapper around the LAN HTTP server.

    ``start`` is non-blocking.  The server intentionally uses a fixed LAN URL
    without application-level authentication; deploy it only on a trusted LAN.
    """

    def __init__(self, controller: Any, host: str = "0.0.0.0", port: int = 8765):
        if not isinstance(host, str) or not host:
            raise ValueError("host must be a non-empty string")
        if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
            raise ValueError("port must be an integer from 0 to 65535")
        self.controller = controller
        self.host = host
        self.port = port
        self._httpd: _ControlHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    @property
    def bound_port(self) -> int | None:
        return self._httpd.server_port if self._httpd is not None else None

    def start(self) -> None:
        with self._lock:
            if self._httpd is not None:
                return
            if not INDEX_FILE.is_file():
                raise FileNotFoundError(f"web UI not found: {INDEX_FILE}")
            httpd = _ControlHTTPServer((self.host, self.port), self)
            thread = threading.Thread(
                target=httpd.serve_forever,
                name="motor-web-control",
                kwargs={"poll_interval": 0.25},
                daemon=True,
            )
            self._httpd = httpd
            self._thread = thread
            thread.start()

    def stop(self) -> None:
        with self._lock:
            httpd, thread = self._httpd, self._thread
            self._httpd = None
            self._thread = None
        if httpd is None:
            return
        httpd.shutdown()
        httpd.server_close()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def __enter__(self) -> "WebControlServer":
        self.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop()
