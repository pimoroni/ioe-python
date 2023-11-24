import pytest


def test_setup(smbus2):
    from ioexpander import IOE

    ioe = IOE(skip_chip_id_check=True)
    del ioe


def test_setup_invalid_chip_id(smbus2):
    from ioexpander import IOE

    smbus2.i2c_msg.read.side_effect = [[0x00, 0x00]]

    with pytest.raises(RuntimeError):
        ioe = IOE()
        del ioe


def test_setup_valid_chip_id(smbus2):
    from ioexpander import IOE

    """
    Using side_effect on i2c_msg returns the
    right values in sequence for our read8 calls:

    msg_r = i2c_msg.read(self._i2c_addr, 1)
    self._i2c_dev.i2c_rdwr(msg_r)
    return list(msg_r)[0]
    """
    smbus2.i2c_msg.read.side_effect = [[0x6A, 0xE2]]

    ioe = IOE()
    del ioe
