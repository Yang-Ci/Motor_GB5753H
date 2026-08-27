import math
import struct
import sys
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from gb5753_tool.canopen import decode_value, parse_sdo_response, sdo_upload_request
from gb5753_tool.object_dictionary import load_xlsx
from gb5753_tool.protocol import (
    CanFrame,
    ControlMode,
    MotorLimits,
    ProtocolError,
    pack_feedback,
    pack_mit,
    pack_position_velocity,
    pack_special,
    pack_velocity,
    parse_feedback,
    parse_hex_data,
    unpack_mit,
)
from gb5753_tool.transport import (
    CANFD_FRAME,
    CAN_FRAME,
    VirtualTransport,
    encode_dm_serial_frame,
    extract_dm_serial_frame,
)


class DaMiaoProtocolTests(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = MotorLimits(12.5, 30.0, 10.0)

    def test_mit_zero_frame_matches_document_bit_layout(self) -> None:
        frame = pack_mit(1, 0, 0, 0, 0, 0, self.limits)
        self.assertEqual(frame.arbitration_id, 0x001)
        self.assertEqual(frame.data, bytes.fromhex("7F FF 7F F0 00 00 07 FF"))

    def test_mit_round_trip_with_quantization(self) -> None:
        source = (1.2, -2.3, 40.0, 0.7, 3.1)
        decoded = unpack_mit(pack_mit(3, *source, self.limits).data, self.limits)
        tolerances = (25 / 65535, 60 / 4095, 500 / 4095, 5 / 4095, 20 / 4095)
        for actual, expected, tolerance in zip(decoded, source, tolerances):
            self.assertLessEqual(abs(actual - expected), tolerance)

    def test_mode_ids_and_little_endian_float_frames(self) -> None:
        pv = pack_position_velocity(2, 1.5, -2.0)
        vel = pack_velocity(2, 3.25)
        self.assertEqual(pv.arbitration_id, 0x102)
        self.assertEqual(pv.data, struct.pack("<ff", 1.5, -2.0))
        self.assertEqual(vel.arbitration_id, 0x202)
        self.assertEqual(vel.data, struct.pack("<f", 3.25))

    def test_special_commands(self) -> None:
        for command, suffix in (("enable", "fc"), ("disable", "fd"), ("set_zero", "fe"), ("clear_error", "fb")):
            frame = pack_special(1, command)
            self.assertEqual(frame.arbitration_id, 0x001)
            self.assertEqual(frame.data.hex(), "ff" * 7 + suffix)

    def test_feedback_round_trip(self) -> None:
        frame = pack_feedback(0x11, 1, 1, 2.0, -3.0, 4.0, self.limits, 40, 41)
        feedback = parse_feedback(frame, self.limits)
        self.assertEqual(feedback.motor_id, 1)
        self.assertEqual(feedback.state_text, "使能")
        self.assertAlmostEqual(feedback.position, 2.0, delta=25 / 65535)
        self.assertAlmostEqual(feedback.velocity, -3.0, delta=60 / 4095)
        self.assertAlmostEqual(feedback.torque, 4.0, delta=20 / 4095)
        self.assertEqual((feedback.mos_temperature, feedback.rotor_temperature), (40, 41))

    def test_out_of_range_rejected_instead_of_wrapping(self) -> None:
        with self.assertRaises(ProtocolError):
            pack_mit(1, 99, 0, 0, 0, 0, self.limits)

    def test_hex_parser(self) -> None:
        self.assertEqual(parse_hex_data("FF 00 1a"), b"\xff\x00\x1a")
        self.assertEqual(parse_hex_data("FF001A"), b"\xff\x00\x1a")
        with self.assertRaises(ProtocolError):
            parse_hex_data("00 " * 9)


class GbCanopenTests(unittest.TestCase):
    def test_sdo_upload_is_fd_brs(self) -> None:
        frame = sdo_upload_request(1, 0x1000, 0)
        self.assertEqual(frame.arbitration_id, 0x601)
        self.assertEqual(frame.data, bytes.fromhex("40 00 10 00 00 00 00 00"))
        self.assertTrue(frame.is_fd)
        self.assertTrue(frame.bitrate_switch)

    def test_sdo_upload_response_decode(self) -> None:
        frame = CanFrame(0x581, bytes.fromhex("43 00 10 00 92 01 02 00"), True, True)
        response = parse_sdo_response(frame)
        self.assertIsNotNone(response)
        assert response is not None
        self.assertEqual((response.index, response.sub_index), (0x1000, 0))
        self.assertEqual(decode_value(response.data, "Uint32"), 0x00020192)

    def test_sdo_abort(self) -> None:
        frame = CanFrame(0x581, bytes.fromhex("80 00 20 00 00 00 02 06"), True, True)
        response = parse_sdo_response(frame)
        assert response is not None
        self.assertEqual(response.abort_code, 0x06020000)

    def test_supplied_object_dictionary(self) -> None:
        dictionary = ROOT / "华翼关节模组对象字典V11(只读).xlsx"
        if not dictionary.exists():
            self.skipTest("厂家对象字典不包含在公开仓库中")
        entries = load_xlsx(dictionary)
        lookup = {(entry.index, entry.sub_index): entry for entry in entries}
        self.assertGreater(len(entries), 350)
        self.assertEqual(lookup[(0x2001, 2)].default_value, "1000000")
        self.assertEqual(lookup[(0x2001, 3)].default_value, "5000000")
        self.assertEqual(lookup[(0x6064, 0)].data_type, "Float32")

    def test_linux_frame_struct_sizes(self) -> None:
        self.assertEqual(CAN_FRAME.size, 16)
        self.assertEqual(CANFD_FRAME.size, 72)


class VirtualTransportTests(unittest.TestCase):
    def test_enable_control_and_feedback(self) -> None:
        limits = MotorLimits()
        bus = VirtualTransport(1, 0x11, limits)
        bus.open()
        bus.send(pack_special(1, "enable"))
        enabled = parse_feedback(bus.recv(0.01), limits)  # type: ignore[arg-type]
        self.assertEqual(enabled.state_code, 1)
        bus.send(pack_mit(1, 1.0, 2.0, 10.0, 0.2, 0.5, limits))
        feedback = parse_feedback(bus.recv(0.01), limits)  # type: ignore[arg-type]
        self.assertTrue(math.isclose(feedback.position, 1.0, abs_tol=0.001))
        self.assertTrue(math.isclose(feedback.velocity, 2.0, abs_tol=0.02))
        bus.close()


class DmSerialTransportTests(unittest.TestCase):
    def test_tx_wrapper_layout(self) -> None:
        packet = encode_dm_serial_frame(CanFrame(0x201, bytes.fromhex("01 02 03 04")))
        self.assertEqual(len(packet), 30)
        self.assertEqual(packet[:3], bytes.fromhex("55 AA 1E"))
        self.assertEqual(packet[13:17], bytes.fromhex("01 02 00 00"))
        self.assertEqual(packet[18], 4)
        self.assertEqual(packet[21:29], bytes.fromhex("01 02 03 04 00 00 00 00"))

    def test_rx_wrapper_resynchronizes(self) -> None:
        packet = bytes.fromhex("AA 11 08 11 00 00 00 10 20 30 40 50 60 23 24 55")
        buffer = bytearray(b"noise" + packet)
        frame = extract_dm_serial_frame(buffer)
        assert frame is not None
        self.assertEqual(frame.arbitration_id, 0x11)
        self.assertEqual(frame.data, bytes.fromhex("10 20 30 40 50 60 23 24"))
        self.assertEqual(buffer, bytearray())


if __name__ == "__main__":
    unittest.main()
