import pytest


def test_setup_switch_counter(smbus2, ioe):
    ioe.setup_switch_counter(1)


def test_read_switch_counter(smbus2, ioe):
    smbus2.i2c_msg.read().__iter__.return_value = [0b10000010]
    assert ioe.read_switch_counter(1) == (2, True)

    smbus2.i2c_msg.read().__iter__.return_value = [0b00000010]
    assert ioe.read_switch_counter(1) == (2, False)


def test_clear_switch_counter(smbus2, ioe):
    ioe.clear_switch_counter(1)
    smbus2.SMBus(1).i2c_rdwr.assert_called_once()


def test_switch_counter_invalid_pin(smbus2, ioe):
    with pytest.raises(ValueError):
        ioe.setup_switch_counter(10)

    with pytest.raises(ValueError):
        ioe.read_switch_counter(10)
