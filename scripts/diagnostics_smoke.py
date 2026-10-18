"""Exercise the actual log/serial/GUI telemetry pipeline using ONLY fake I/O.

All values and deliberate exceptions from this script are SYNTHETIC, not a
hardware acceptance test or a collision assessment. No serial.Serial is opened.
Run with the same Python/Tk dependencies as pc_gui.py. Logs remain under logs/.
"""
import argparse
import json
from pathlib import Path
import queue
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motor_control import BindingSet, LogicalRole
from motor_control.desktop_app import StepperGUI
from motor_control.diagnostics import DiagnosticRecorder, environment_summary, operation_scope
from motor_control.diagnostic_export import export_bundle
from motor_control.serial_session import SerialSession


class ScriptedPort:
    """In-memory peer; commands never leave this process."""
    def __init__(self):
        self.is_open = True
        self.rx = queue.Queue()
        self.sample = 0

    def readline(self):
        try:
            return self.rx.get(timeout=.02)
        except queue.Empty:
            return b""

    def write(self, payload):
        fields = payload.decode("ascii").strip().split(",")
        if fields[:2] == ["X42S", "S"]:
            driver = int(fields[2])
            if driver == 9:
                return len(payload)  # Deliberate missing reply; read-only timeout.
            motor_deg = (1 if driver in (1, 2) else -1) * self.sample * 21.5
            reply = f"X42S,{driver},S,1,1,1,none,{motor_deg},{motor_deg},0,0,0,25,5,0,7"
        elif fields[0] == "X42S" and fields[2] == "MOVE":
            reply = "ACK," + ",".join(fields)
        else:
            raise ValueError(f"Unsupported synthetic command: {fields}")
        self.rx.put((reply + "\n").encode("ascii"))
        return len(payload)

    def close(self):
        self.is_open = False
        self.rx.put(b"")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="optional redacted ZIP output")
    args = parser.parse_args()
    recorder = DiagnosticRecorder(ROOT, metadata={**environment_summary(ROOT),
        "origin": "offline-synthetic-diagnostics-smoke", "hardware_connected": False,
        "warning": "Synthetic positions/ACK/timeout/error; not field measurements or hardware acceptance."})
    # Minimal GUI state invokes the production telemetry merger/converter. It
    # deliberately does not run __init__, create a window, or load local config.
    app = StepperGUI.__new__(StepperGUI)
    app.diagnostics = recorder
    app.state_lock = threading.Lock()
    app._x42s_states = {}
    app.control_bindings = BindingSet.from_driver_id_mapping(
        {"Mup1": 1, "Mr1": 2, "Mup2": 3, "Mr2": 4},
        lead_mms={"Mup1": 8.0, "Mup2": 8.0})
    port = ScriptedPort()
    try:
        recorder.event("gui", "configuration.synthetic", bindings=app.control_bindings.as_document())
        with SerialSession(port, diagnostics=recorder) as session:
            with operation_scope("offline-demo-four-axis"):
                recorder.event("gui", "action.synthetic", note="in-memory serial only")
                reply = session.request("X42S,2,MOVE,191,191")
                assert reply.steps == 191
                for index in range(5):
                    port.sample = index
                    for role in LogicalRole:
                        driver = app.control_bindings.for_role(role).driver_id
                        reply = session.request(f"X42S,S,{driver}")
                        app._apply_x42s_reply(role, driver, reply)
                assert session.request("X42S,S,9", timeout=.02, soft_timeout=True) is None
                try:
                    raise ValueError("Intentional offline GUI exception; not an application fault")
                except ValueError as exc:
                    recorder.exception("gui", "validation.synthetic_error", exc)
        assert recorder.flush()
        samples = [json.loads(line) for line in
                   (recorder.directory / "telemetry.jsonl").read_text(encoding="utf-8").splitlines()]
        assert len(samples) == 20
        assert samples[-1]["data"]["output_position"] == -4.0
        health = recorder.health()
        assert not health["dropped"] and not health["write_errors"], health
        recorder.event("gui", "validation.passed", telemetry_rows=len(samples), hardware_connected=False)
    finally:
        recorder.close()
    if args.output:
        print(export_bundle(ROOT, args.output, recorder=recorder,
              label="OFFLINE validation + recent local logs (legacy logs are historical, not X42S acceptance)"))
    print(f"PASS: {recorder.session_id}, 20 synthetic telemetry rows; no hardware connected")


if __name__ == "__main__":
    main()
