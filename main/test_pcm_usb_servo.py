#!/usr/bin/env python3
import struct
import threading
import unittest
from unittest.mock import Mock

from pcm_usb_servo import (
    MSG_SDO_RSP,
    PcmUsbError,
    PcmUsbServoClient,
    SESSION_COMMAND_GUARD,
    SessionStatus,
    build_dop_frame,
    build_live_begin_command,
    build_sdo_write,
    cobs_decode,
    cobs_encode,
    parse_dop_frame,
    parse_session_hello,
    parse_session_status,
    parse_sdo_response,
)


class ProtocolTest(unittest.TestCase):
    def test_cobs_round_trip(self):
        raw = b"\x00\x01\x02\x00\xff\x00"
        self.assertEqual(cobs_decode(cobs_encode(raw)), raw)

    def test_servo_on_sequence_zero_matches_studio(self):
        frame = build_sdo_write(0x5F08, 1, b"\x01", 0)
        self.assertEqual(
            frame.hex(" "),
            "03 01 51 01 06 21 08 5f 01 01 04 01 34 7f 00")

    def test_servo_off_sequence_zero_matches_studio(self):
        frame = build_sdo_write(0x5F08, 1, b"\x00", 0)
        self.assertEqual(
            frame.hex(" "),
            "03 01 51 01 06 21 08 5f 01 01 01 03 15 6f 00")

    def test_parse_explicit_length_state_response(self):
        payload = struct.pack("<BHBH", 0x41, 0x5F08, 2, 1) + b"\x01"
        encoded = build_dop_frame(MSG_SDO_RSP, payload, 19)[:-1]
        msg_type, node_id, sequence, decoded_payload = parse_dop_frame(encoded)
        response = parse_sdo_response(decoded_payload)
        self.assertEqual((msg_type, node_id, sequence), (MSG_SDO_RSP, 0x51, 19))
        self.assertIsNotNone(response)
        self.assertEqual(response.data, b"\x01")

    def test_parse_write_ack(self):
        response = parse_sdo_response(struct.pack("<BHB", 0x60, 0x5F08, 1))
        self.assertTrue(response.is_write_ack)

    def test_parse_abort(self):
        payload = struct.pack("<BHBI", 0x80, 0x5F08, 1, 0x06020000)
        response = parse_sdo_response(payload)
        self.assertTrue(response.is_abort)
        self.assertEqual(response.abort_code, 0x06020000)

    def test_build_live_begin_command_matches_studio_abi(self):
        payload = build_live_begin_command(
            session_epoch=0x50FFFE3D,
            generation=7,
            nonce=0x1122334455667788)
        self.assertEqual(
            struct.unpack("<HBBIIIII", payload),
            (1, 1, 1, 0x50FFFE3D, 7, 0x55667788, 0x11223344,
             SESSION_COMMAND_GUARD))

    def test_parse_session_hello_and_status(self):
        hello = parse_session_hello(struct.pack(
            "<HBBII", 1, 1, 1, 15, 0x50FFFE3D))
        self.assertEqual(hello.session_epoch, 0x50FFFE3D)
        status = parse_session_status(struct.pack(
            "<HBBBBHIIIII", 1, 4, 5, 1, 0, 62,
            0x50FFFE3D, 7, 9, 11, 13))
        self.assertEqual((status.mode, status.phase, status.flags), (4, 5, 62))

    def test_cached_live_session_is_revalidated(self):
        client = PcmUsbServoClient.__new__(PcmUsbServoClient)
        client._studio_live = True
        client._stop = threading.Event()
        client._read_session_status = Mock(return_value=SessionStatus(
            1, 4, 5, 1, 0, 0, 1, 1, 0, 0, 0))

        client._ensure_studio_live("/dev/ttyACM0")

        client._read_session_status.assert_called_once_with()
        self.assertTrue(client._studio_live)

    def test_stale_live_session_enters_fresh_setup(self):
        client = PcmUsbServoClient.__new__(PcmUsbServoClient)
        client._studio_live = True
        client._stop = threading.Event()
        client._read_session_status = Mock(return_value=SessionStatus(
            1, 2, 0, 0, 0, 25, 1, 0, 0, 0, 1))
        client._publish = Mock()
        client._pcm_mounted_block_devices = Mock(
            side_effect=PcmUsbError("fresh setup reached"))

        with self.assertRaisesRegex(PcmUsbError, "fresh setup reached"):
            client._ensure_studio_live("/dev/ttyACM0")

        self.assertFalse(client._studio_live)

    def test_servo_on_releases_usb_owner_after_state_confirmation(self):
        client = PcmUsbServoClient.__new__(PcmUsbServoClient)
        client._stop = threading.Event()
        client._unarm_requested = threading.Event()
        client._release_port_until_command = False
        client._port_release_not_before = 0.0
        client._ensure_studio_live = Mock()
        client._exchange = Mock(return_value=type(
            "WriteAck", (), {"is_abort": False, "is_write_ack": True})())
        client._read_servo_state = Mock(return_value=1)
        client._publish = Mock()

        client._perform_servo_on("/dev/ttyACM0")

        self.assertTrue(client._release_port_until_command)
        client._publish.assert_called_with(
            connected=True, port="/dev/ttyACM0", phase="ON", servo_state=1,
            message=("ARM 완료 — PCM 서보 ON 확인 · "
                     "모션 소유권 반환을 위해 USB 세션 해제"))

    def test_servo_off_releases_stale_cdc_after_state_confirmation(self):
        client = PcmUsbServoClient.__new__(PcmUsbServoClient)
        client._stop = threading.Event()
        client._release_port_until_command = False
        client._port_release_not_before = 0.0
        client._prepare_servo_off_channel = Mock()
        client._exchange_retry = Mock(return_value=type(
            "WriteAck", (), {"is_abort": False, "is_write_ack": True})())
        client._read_servo_state = Mock(return_value=0)
        client._publish = Mock()

        client._perform_servo_off("/dev/ttyACM0")

        client._prepare_servo_off_channel.assert_called_once_with(
            "/dev/ttyACM0")
        self.assertTrue(client._release_port_until_command)
        self.assertGreater(client._port_release_not_before, 0.0)
        client._publish.assert_called_with(
            connected=True, port="/dev/ttyACM0", phase="CONNECTED",
            servo_state=0,
            message=("UNARM 완료 — PCM 서보 OFF 확인 · "
                     "다음 ARM 전까지 USB 포트 해제"))


if __name__ == "__main__":
    unittest.main()
