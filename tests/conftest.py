import sys

import mock
import pytest


@pytest.fixture(scope="function", autouse=True)
def cleanup():
    """This fixture removes modules under test from sys.modules.

    This ensures that each module is fully re-imported, along with
    the fixtures for each test function.

    """

    yield None
    try:
        del sys.modules["ioexpander"]
    except KeyError:
        pass


@pytest.fixture(scope="function", autouse=False)
def smbus2():
    """Mock smbus2 module."""

    smbus2 = mock.MagicMock()
    smbus2.i2c_msg.read().__iter__.return_value = [0b00000000]
    sys.modules["smbus2"] = smbus2
    yield smbus2
    del sys.modules["smbus2"]


@pytest.fixture(scope="function")
def ioe():
    from ioexpander import IOE

    yield IOE(skip_chip_id_check=True)
    del sys.modules["ioexpander"]
