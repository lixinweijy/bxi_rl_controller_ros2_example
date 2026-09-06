import struct

from bxi_example_py_elf3 import motor_online_test as tool


def test_probe_and_response_frames():
    frame = tool._probe_frame(1)
    can_id, length, flags, _, _, payload = tool._CANFD_FRAME.unpack(frame)
    assert can_id == 0x7FF
    assert length == 64
    assert flags == tool._CANFD_BRS | tool._CANFD_FDF
    assert payload[:56] == tool._EXIT_MOTOR_MODE * 7
    assert payload[56:] == b"\0" * 8

    response = tool._CANFD_FRAME.pack(3, 8, 0, 0, 0, bytes(64))
    assert tool._response_motor_id(response) == 3
    assert tool._response_motor_id(struct.pack("=IB3x8s", 0x7FF, 8, bytes(8))) is None
