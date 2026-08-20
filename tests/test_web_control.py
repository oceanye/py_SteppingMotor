import http.client
import json
import unittest

from web_control import WebControlServer


class FakeWebController:
    def __init__(self):
        self.calls = []

    def web_get_status(self):
        return {"connected": True, "topology": {"total_axes": 30}}

    def web_stepper_move(self, axis, direction, distance_mm, speed_mm_s=3.0):
        self.calls.append(("move", axis, direction, distance_mm, speed_mm_s))
        return {"ok": True, "axis": axis}

    def web_stepper_stop(self, axis):
        self.calls.append(("stop", axis))
        return {"ok": True}

    def web_stepper_config(self, axis, **kwargs):
        self.calls.append(("config", axis, kwargs))
        return {"ok": True, "axis": axis, **kwargs}

    def web_motor_command(self, mode, axis, action, target_deg=None):
        self.calls.append(("motor", mode, axis, action, target_deg))
        return {"ok": True}

    def web_track_command(self, action, pwm=0, lease_ms=0):
        self.calls.append(("track", action, pwm, lease_ms))
        return {"ok": True}

    def web_emergency_stop(self):
        self.calls.append(("estop",))
        return {"ok": True, "confirmed": True}


class WebControlTests(unittest.TestCase):
    def setUp(self):
        self.controller = FakeWebController()
        self.server = WebControlServer(self.controller, host="127.0.0.1", port=0)
        self.server.start()
        self.connection = http.client.HTTPConnection(
            "127.0.0.1", self.server.bound_port, timeout=2.0)

    def tearDown(self):
        self.connection.close()
        self.server.stop()

    def request_json(self, method, path, body=None):
        payload = None if body is None else json.dumps(body).encode("utf-8")
        headers = {} if payload is None else {
            "Content-Type": "application/json",
            "Content-Length": str(len(payload)),
        }
        self.connection.request(method, path, body=payload, headers=headers)
        response = self.connection.getresponse()
        data = json.loads(response.read().decode("utf-8"))
        return response.status, data

    def test_status_exposes_current_30_axis_topology(self):
        status, data = self.request_json("GET", "/api/status")
        self.assertEqual(status, 200)
        self.assertEqual(data["topology"]["total_axes"], 30)

    def test_axis_config_for_last_remote_axis_reaches_controller(self):
        status, data = self.request_json(
            "POST",
            "/api/stepper/config",
            {"axis": 29, "mode": "rotary", "gear_ratio": 36.0},
        )
        self.assertEqual(status, 200)
        self.assertEqual(data["mode"], "rotary")
        self.assertEqual(
            self.controller.calls,
            [("config", 29, {"mode": "rotary", "gear_ratio": 36.0})],
        )

    def test_axis_30_is_rejected_before_controller_call(self):
        status, data = self.request_json(
            "POST", "/api/stepper/stop", {"axis": 30})
        self.assertEqual(status, 400)
        self.assertFalse(data["ok"])
        self.assertEqual(self.controller.calls, [])

    def test_unknown_fields_are_rejected(self):
        status, data = self.request_json(
            "POST", "/api/estop", {"unexpected": True})
        self.assertEqual(status, 400)
        self.assertFalse(data["ok"])
        self.assertEqual(self.controller.calls, [])


if __name__ == "__main__":
    unittest.main()
