import pytest


def test_out_of_range_pins(smbus2):
    from ioexpander import IOE, PIN_MODE_IN

    ioe = IOE(skip_chip_id_check=True)

    with pytest.raises(ValueError):
        ioe.set_mode(0, PIN_MODE_IN)

    with pytest.raises(ValueError):
        ioe.set_mode(15, PIN_MODE_IN)


def test_in_range_pins(smbus2):
    from ioexpander import IOE, PIN_MODE_IN

    ioe = IOE(skip_chip_id_check=True)

    for pin in range(1, 15):
        ioe.set_mode(pin, PIN_MODE_IN)


def test_adc_input(smbus2):
    from ioexpander import IOE, PIN_MODE_ADC

    ioe = IOE(skip_chip_id_check=True)

    ioe.set_mode(7, PIN_MODE_ADC)
    assert ioe.get_mode(7) == PIN_MODE_ADC

    # Always return the high bit set, this pretends bit 7 of REG_ADCCON0 is set
    # and allows an "ADC read" to complete without a timeout exception.
    smbus2.i2c_msg.read().__iter__.return_value = [0b10000000, 0b10000000]

    result = ioe.input(7)
    assert isinstance(result, float)

    # (128 << 4) | 128 / 4095.0 * 3.3
    # round to 2dp to account for FLOATING POINT WEIRDNESS!
    assert round(result, 2) == 1.75

    ioe.set_adc_vref(5.0)

    result = ioe.input(7)
    # (128 << 4) | 128 / 4095.0 * 5.0
    assert round(result, 2) == 2.66


def test_adc_input_timeout_should_raise_runtimeerror(smbus2):
    from ioexpander import IOE, PIN_MODE_ADC

    ioe = IOE(skip_chip_id_check=True)

    ioe.set_mode(7, PIN_MODE_ADC)
    assert ioe.get_mode(7) == PIN_MODE_ADC

    # Always return the high bit cleared, this pretends bit 7 of REG_ADCCON0 is cleared
    # and causes an "ADC read" to raise a timeout exception.
    smbus2.i2c_msg.read().__iter__.return_value = [0b00000000]

    with pytest.raises(RuntimeError):
        ioe.input(7, adc_timeout=0.1)


def test_gpio_input(smbus2):
    from ioexpander import HIGH, IOE, LOW, PIN_MODE_IN

    ioe = IOE(skip_chip_id_check=True)

    ioe.set_mode(1, PIN_MODE_IN)
    assert ioe.get_mode(1) == PIN_MODE_IN

    # Always LOW result
    smbus2.i2c_msg.read().__iter__.return_value = [0b00000000]
    assert ioe.input(1) == LOW

    # Always HIGH result
    smbus2.i2c_msg.read().__iter__.return_value = [0b11111111]
    assert ioe.input(1) == HIGH


def test_gpio_input_pull_up(smbus2):
    from ioexpander import HIGH, IOE, LOW, PIN_MODE_PU

    ioe = IOE(skip_chip_id_check=True)

    ioe.set_mode(1, PIN_MODE_PU)
    assert ioe.get_mode(1) == PIN_MODE_PU

    # Always LOW result
    smbus2.i2c_msg.read().__iter__.return_value = [0b00000000]
    assert ioe.input(1) == LOW

    # Always HIGH result
    smbus2.i2c_msg.read().__iter__.return_value = [0b11111111]
    assert ioe.input(1) == HIGH


def test_pwm_set_mode(smbus2):
    from ioexpander import IOE, PIN_MODE_PWM

    ioe = IOE(skip_chip_id_check=True)

    ioe.set_mode(1, PIN_MODE_PWM)


def test_non_pwm_set_mode_should_raise_valueerror(smbus2):
    from ioexpander import IOE, PIN_MODE_PWM

    ioe = IOE(skip_chip_id_check=True)

    with pytest.raises(ValueError):
        ioe.set_mode(10, PIN_MODE_PWM)


def test_set_pwm_control(smbus2):
    from ioexpander import IOE, PIN_MODE_PWM

    ioe = IOE(skip_chip_id_check=True)

    ioe.set_mode(1, PIN_MODE_PWM)
    ioe.set_pwm_control(divider=8)
