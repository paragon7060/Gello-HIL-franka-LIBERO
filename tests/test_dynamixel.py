import pytest

from gello_hil.dynamixel import PING, READ, SYNC_READ, protocol2_packet


@pytest.mark.parametrize("instruction", [PING, READ])
def test_read_only_packets_allow_single_motor_reads(instruction):
    assert protocol2_packet(1, instruction).startswith(bytes((0xFF, 0xFF, 0xFD, 0x00)))


def test_sync_read_requires_broadcast_id():
    with pytest.raises(ValueError):
        protocol2_packet(1, SYNC_READ)
    assert protocol2_packet(0xFE, SYNC_READ)


@pytest.mark.parametrize("write_instruction", [0x03, 0x04, 0x05, 0x83, 0x93])
def test_every_write_class_instruction_is_rejected(write_instruction):
    with pytest.raises(ValueError, match="read-only"):
        protocol2_packet(1, write_instruction)
