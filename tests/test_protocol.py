import unittest

from motor_control.protocol import (
    AckReply,
    EncoderStatusReply,
    ErrorReply,
    FirmwareMode,
    HardwareEstopEvent,
    HardwareEstopState,
    MAX_STEP_DELAY_US,
    ModeReply,
    MotorFaultEvent,
    MotorStatusReply,
    NodeOfflineEvent,
    NodeStatusReply,
    OkReply,
    ProtocolEncodingError,
    StepProgress,
    StepResult,
    StepTerminal,
    TrackDirection,
    TrackStatusReply,
    TrackTimeout,
    UnknownMessage,
    build_encoder_status_command,
    build_estop_command,
    build_foc_command,
    build_mode_command,
    build_move_command,
    build_node_status_command,
    build_stop_command,
    build_track_command,
    encode_line,
    parse_line,
    reply_matcher_for,
)


class CommandEncodingTests(unittest.TestCase):
    def test_existing_command_wire_formats_are_preserved(self):
        self.assertEqual(build_move_command(6, 1200, True, 100), "MOVE,6,1200,1,100")
        self.assertEqual(build_move_command("*", 3, 0, 50), "MOVE,*,3,0,50")
        self.assertEqual(build_stop_command(29), "STOP,29")
        self.assertEqual(build_stop_command("*"), "STOP,*")
        self.assertEqual(build_estop_command(), "ESTOP")
        self.assertEqual(build_mode_command(), "MODE")
        self.assertEqual(build_foc_command(1, "pa", "12.50"), "FOC,1,PA,12.50")
        self.assertEqual(build_track_command("fwd", 60, 1500), "TRACK,D,FWD,60,1500")
        self.assertEqual(build_node_status_command(), "NODE,*,S")
        self.assertEqual(build_node_status_command(2), "NODE,2,S")
        self.assertEqual(build_encoder_status_command(5), "ENC,5,S")
        self.assertEqual(encode_line("MODE"), b"MODE\n")

    def test_invalid_or_injected_command_fields_are_rejected(self):
        with self.assertRaises(ProtocolEncodingError):
            build_move_command(0, 0, 0, 100)
        with self.assertRaises(ProtocolEncodingError):
            build_move_command(0, 1, 2, 100)
        with self.assertRaises(ProtocolEncodingError):
            build_foc_command(0, "S\nESTOP")
        with self.assertRaises(ProtocolEncodingError):
            encode_line("MODE\r\nESTOP")

    def test_long_rotary_delays_match_firmware_contract(self):
        self.assertEqual(
            build_move_command(0, 1, 1, 6_000_000),
            "MOVE,0,1,1,6000000",
        )
        self.assertEqual(MAX_STEP_DELAY_US, 10_000_000)
        with self.assertRaises(ProtocolEncodingError):
            build_move_command(0, 1, 1, MAX_STEP_DELAY_US + 1)


class EventParsingTests(unittest.TestCase):
    def test_step_progress_done_abort_and_legacy_terminal(self):
        progress = parse_line(b"STEP,6,P,250,1000\r\n")
        self.assertEqual(progress, StepProgress("STEP,6,P,250,1000", 6, 250, 1000))

        done = parse_line("STEP,29,DONE,1000,1000")
        self.assertEqual(done.result, StepResult.DONE)
        self.assertEqual((done.axis, done.executed_steps, done.requested_steps), (29, 1000, 1000))

        abort = parse_line("STEP,8,ABORT,123,1000")
        self.assertEqual(abort.result, StepResult.ABORT)
        self.assertEqual((abort.executed_steps, abort.requested_steps), (123, 1000))

        legacy = parse_line("STEP,0,DONE")
        self.assertEqual(legacy, StepTerminal("STEP,0,DONE", 0, StepResult.DONE, None, None))

    def test_safety_track_fault_and_node_offline_events(self):
        self.assertEqual(parse_line("TRACK,D,TIMEOUT"), TrackTimeout("TRACK,D,TIMEOUT"))
        self.assertEqual(
            parse_line("HWESTOP,TRIGGERED"),
            HardwareEstopEvent("HWESTOP,TRIGGERED", HardwareEstopState.TRIGGERED),
        )
        self.assertEqual(
            parse_line("HWESTOP,CLEARED"),
            HardwareEstopEvent("HWESTOP,CLEARED", HardwareEstopState.CLEARED),
        )
        self.assertEqual(parse_line("FOC,1,FAULT"), MotorFaultEvent("FOC,1,FAULT", 1, ()))
        self.assertEqual(parse_line("NODE,4,OFFLINE"), NodeOfflineEvent("NODE,4,OFFLINE", 4))


class ReplyParsingTests(unittest.TestCase):
    def test_basic_replies_and_estop_ack(self):
        self.assertEqual(parse_line("ACK,6"), AckReply("ACK,6", 6))
        self.assertEqual(parse_line("OK,ESTOP"), OkReply("OK,ESTOP", ("ESTOP",)))
        self.assertEqual(parse_line("ERR:hardware estop active"), ErrorReply("ERR:hardware estop active", "hardware estop active"))
        self.assertEqual(parse_line("MODE,GEAR"), ModeReply("MODE,GEAR", FirmwareMode.GEAR))

    def test_typed_status_replies(self):
        motor = parse_line("FOC,1,S,2,-10.5,20.0,1")
        self.assertEqual(motor, MotorStatusReply("FOC,1,S,2,-10.5,20.0,1", 1, 2, -10.5, 20.0, True))

        track = parse_line("TRACK,D,S,REV,55,800")
        self.assertEqual(track, TrackStatusReply("TRACK,D,S,REV,55,800", TrackDirection.REVERSE, 55, 800))

        encoder = parse_line("ENC,0,S,1,1,1,1,2048,180.00,540.00,5,2")
        self.assertIsInstance(encoder, EncoderStatusReply)
        self.assertEqual((encoder.raw_angle, encoder.multi_turn_deg, encoder.error_count), (2048, 540.0, 2))

        online = parse_line("NODE,2,S,ONLINE,5,17")
        offline = parse_line("NODE,3,S,OFFLINE,0,0")
        self.assertEqual(online, NodeStatusReply("NODE,2,S,ONLINE,5,17", 2, True, 5, 17))
        self.assertEqual(offline, NodeStatusReply("NODE,3,S,OFFLINE,0,0", 3, False, 0, 0))

    def test_unknown_and_malformed_lines_are_not_typed_as_replies(self):
        samples = (
            "ESP32 Controller Ready",
            "Protocol: MOVE/STOP/ESTOP",
            "STEP,0,WHAT,1,2",
            "STEP,x,DONE,1,1",
            "NODE,2,OFFLINE,extra",
            "NODE,2,S,MAYBE,0,0",
            "ACK,no-axis",
            "MODE,UNKNOWN",
            "FOC,0,S,1,nan,0,0",
        )
        for sample in samples:
            with self.subTest(sample=sample):
                self.assertIsInstance(parse_line(sample), UnknownMessage)

    def test_command_specific_reply_matchers_reject_unrelated_known_reply(self):
        mode_matcher = reply_matcher_for("MODE")
        self.assertTrue(mode_matcher(parse_line("MODE,FOC")))
        self.assertFalse(mode_matcher(parse_line("ACK,0")))

        move_matcher = reply_matcher_for("MOVE,6,10,1,100")
        self.assertTrue(move_matcher(parse_line("ACK,6")))
        self.assertFalse(move_matcher(parse_line("ACK,5")))
        self.assertTrue(move_matcher(parse_line("ERR:busy")))

        node_matcher = reply_matcher_for("NODE,3,S")
        self.assertTrue(node_matcher(parse_line("NODE,3,S,ONLINE,0,1")))
        self.assertFalse(node_matcher(parse_line("NODE,2,S,ONLINE,0,1")))


if __name__ == "__main__":
    unittest.main()
