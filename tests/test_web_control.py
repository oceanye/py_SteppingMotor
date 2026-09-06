import http.client
import json
import unittest

from web_control import WebControlServer


class FakeWebController:
    def __init__(self):
        self.calls = []

    def web_get_status(self):
        return {"connected": True, "topology": {"total_axes": 30}}

    def web_stepper_move(
        self,
        axis,
        direction,
        distance_mm,
        speed_mm_s=3.0,
        confirm_high_rate=False,
    ):
        self.calls.append(
            (
                "move",
                axis,
                direction,
                distance_mm,
                speed_mm_s,
                confirm_high_rate,
            )
        )
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

    def request(self, method, path, body=None):
        payload = None if body is None else json.dumps(body).encode("utf-8")
        headers = {} if payload is None else {
            "Content-Type": "application/json",
            "Content-Length": str(len(payload)),
        }
        self.connection.request(method, path, body=payload, headers=headers)
        response = self.connection.getresponse()
        response_headers = {
            name.lower(): value for name, value in response.getheaders()
        }
        return response.status, response_headers, response.read()

    def request_json(self, method, path, body=None):
        status, _headers, content = self.request(method, path, body)
        return status, json.loads(content.decode("utf-8"))

    def test_index_is_served_with_control_page_and_security_headers(self):
        status, headers, content = self.request("GET", "/index.html")

        self.assertEqual(status, 200)
        self.assertTrue(headers["content-type"].startswith("text/html"))
        self.assertEqual(headers["cache-control"], "no-store")
        self.assertEqual(headers["referrer-policy"], "no-referrer")
        self.assertEqual(headers["x-content-type-options"], "nosniff")
        self.assertEqual(headers["x-frame-options"], "DENY")
        self.assertIn("default-src 'self'", headers["content-security-policy"])
        self.assertIn("frame-ancestors 'none'", headers["content-security-policy"])
        self.assertNotIn("access-control-allow-origin", headers)

        page = content.decode("utf-8")
        self.assertIn("<!doctype html>", page.lower())
        self.assertIn('id="estop"', page)
        self.assertIn("/api/status", page)
        self.assertIn('id="speedPresets"', page)
        self.assertIn('id="sppr${a}"', page)
        self.assertIn('id="slead${a}"', page)
        self.assertIn("confirm_high_rate:highRate", page)
        self.assertIn("axisConfigIsDirty(axis)", page)
        self.assertIn('id="logicalMotors"', page)
        self.assertIn("只读监控", page)
        self.assertIn("host_pulse_accounting", page)
        self.assertIn("refreshInFlight", page)
        self.assertIn("typeof position==='number'", page)
        self.assertIn("item.target_position", page)

    def test_logical_binding_write_and_auto_start_routes_do_not_exist(self):
        for path in ("/api/logical-motors/bindings", "/api/coordinated/start"):
            with self.subTest(path=path):
                status, data = self.request_json("POST", path, {})
                self.assertEqual(status, 404)
                self.assertIn("未知路径", data["error"])

    def test_status_exposes_current_30_axis_topology(self):
        status, data = self.request_json("GET", "/api/status")
        self.assertEqual(status, 200)
        self.assertEqual(data["topology"]["total_axes"], 30)

    def test_axis_config_for_last_remote_axis_reaches_controller(self):
        status, data = self.request_json(
            "POST",
            "/api/stepper/config",
            {
                "axis": 29,
                "mode": "rotary",
                "pulse_per_rev": 400,
                "gear_ratio": 36.0,
                "lead_mm": 5.0,
            },
        )
        self.assertEqual(status, 200)
        self.assertEqual(data["mode"], "rotary")
        self.assertEqual(
            self.controller.calls,
            [
                (
                    "config",
                    29,
                    {
                        "mode": "rotary",
                        "pulse_per_rev": 400.0,
                        "gear_ratio": 36.0,
                        "lead_mm": 5.0,
                    },
                )
            ],
        )

    def test_stepper_move_passes_optional_high_rate_confirmation(self):
        status, data = self.request_json(
            "POST",
            "/api/stepper/move",
            {
                "axis": 6,
                "direction": "forward",
                "distance_mm": 2.5,
                "speed_mm_s": 3.0,
                "confirm_high_rate": True,
            },
        )

        self.assertEqual(status, 200)
        self.assertTrue(data["ok"])
        self.assertEqual(
            self.controller.calls,
            [("move", 6, "forward", 2.5, 3.0, True)],
        )

    def test_stepper_move_defaults_high_rate_confirmation_to_false(self):
        status, data = self.request_json(
            "POST",
            "/api/stepper/move",
            {
                "axis": 0,
                "direction": "reverse",
                "distance_mm": 1.0,
                "speed_mm_s": 2.0,
            },
        )

        self.assertEqual(status, 200)
        self.assertTrue(data["ok"])
        self.assertEqual(
            self.controller.calls,
            [("move", 0, "reverse", 1.0, 2.0, False)],
        )

    def test_stepper_move_rejects_non_boolean_high_rate_confirmation(self):
        for value in (0, 1, "true", None, [], {}):
            with self.subTest(value=value):
                status, data = self.request_json(
                    "POST",
                    "/api/stepper/move",
                    {
                        "axis": 0,
                        "direction": "forward",
                        "distance_mm": 1.0,
                        "speed_mm_s": 2.0,
                        "confirm_high_rate": value,
                    },
                )
                self.assertEqual(status, 400)
                self.assertFalse(data["ok"])
        self.assertEqual(self.controller.calls, [])

    def test_stepper_stop_success_reaches_controller(self):
        status, data = self.request_json(
            "POST", "/api/stepper/stop", {"axis": 29}
        )

        self.assertEqual(status, 200)
        self.assertTrue(data["ok"])
        self.assertEqual(self.controller.calls, [("stop", 29)])

    def test_motor_target_success_reaches_controller(self):
        status, data = self.request_json(
            "POST",
            "/api/motor",
            {"mode": "FOC", "axis": 1, "action": "target", "target_deg": 12.5},
        )

        self.assertEqual(status, 200)
        self.assertTrue(data["ok"])
        self.assertEqual(
            self.controller.calls,
            [("motor", "FOC", 1, "target", 12.5)],
        )

    def test_track_run_and_stop_success_reach_controller(self):
        status, data = self.request_json(
            "POST",
            "/api/track",
            {"action": "forward", "pwm": 60, "lease_ms": 1_000},
        )
        self.assertEqual(status, 200)
        self.assertTrue(data["ok"])

        status, data = self.request_json(
            "POST", "/api/track", {"action": "stop"}
        )
        self.assertEqual(status, 200)
        self.assertTrue(data["ok"])
        self.assertEqual(
            self.controller.calls,
            [("track", "forward", 60.0, 1_000), ("track", "stop", 0.0, 0)],
        )

    def test_estop_success_reaches_controller(self):
        status, data = self.request_json("POST", "/api/estop", {})

        self.assertEqual(status, 200)
        self.assertEqual(data, {"ok": True, "confirmed": True})
        self.assertEqual(self.controller.calls, [("estop",)])

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
