import time

from smbus2 import SMBus, i2c_msg

from . import sioe_regs
from . import ioe_regs

__version__ = '0.0.4'


# These values encode our desired pin function: IO, ADC, PWM
# alongside the GPIO MODE for that port and pin (section 8.1)
# the 5th bit additionally encodes the default output state
PIN_MODE_IO = 0b00000   # General IO mode, IE: not ADC or PWM
PIN_MODE_QB = 0b00000   # Output, Quasi-Bidirectional mode
PIN_MODE_PP = 0b00001   # Output, Push-Pull mode
PIN_MODE_IN = 0b00010   # Input-only (high-impedance)
PIN_MODE_PU = 0b10000   # Input (with pull-up)
PIN_MODE_OD = 0b00011   # Output, Open-Drain mode
PIN_MODE_PWM = 0b00101  # PWM, Output, Push-Pull mode
PIN_MODE_ADC = 0b01010  # ADC, Input-only (high-impedance)
MODE_NAMES = ("IO", "PWM", "ADC")
GPIO_NAMES = ("QB", "PP", "IN", "OD")
STATE_NAMES = ("LOW", "HIGH")

# More convinient names for the pin functions
IN = PIN_MODE_IN
IN_PULL_UP = PIN_MODE_PU
IN_PU = PIN_MODE_PU
OUT = PIN_MODE_PP
PWM = PIN_MODE_PWM
ADC = PIN_MODE_ADC

HIGH = 1
LOW = 0

CLOCK_FREQ = 24000000
MAX_PERIOD = (1 << 16) - 1
MAX_DIVIDER = (1 << 7)


class PIN:
    """Pin.

    Class to store details of an IO pin.

    """

    def __init__(self, port=None, pin=None, enc_channel=None):
        if getattr(self, "type", None) is None:
            self.type = [PIN_MODE_IO]
        self.mode = None
        self.port = port
        self.pin = pin
        self.enc_channel = enc_channel
        self.invert_output = False

    def is_inverted(self):
        return self.invert_output

    def set_inverted(self, invert):
        self.invert_output = invert


class PWM_PIN(PIN):
    """PWM Pin.

    Class to store details of a PWM-enabled pin.

    """

    def __init__(self, port=None, pin=None, pwm_piocon=None, pwm_define=None, enc_channel=None):
        PIN.__init__(self, port, pin, enc_channel)
        self.type.append(PIN_MODE_PWM)
        self.reg_iopwm, self.bit_iopwm = pwm_piocon
        self.pwm_module, self.pwm_channel = pwm_define


class DUAL_PWM_PIN(PWM_PIN):
    """PWM Pin, with alt.

    Class to store details of a PWM-enabled pin, with alt.

    """

    def __init__(self, port=None, pin=None, pwm_piocon=None, pwm_define=None, pwm_auxr=None, pwm_alt_define=None, enc_channel=None):
        PWM_PIN.__init__(self, port, pin, pwm_piocon, pwm_define, enc_channel)
        self.type.append(PIN_MODE_PWM)
        self.reg_auxr, self.bit_auxr, self.val_auxr = pwm_auxr
        self.pwm_alt_module, self.pwm_alt_channel = pwm_alt_define
        self.using_alt = False

    def is_using_alt(self):
        return self.using_alt

    def set_using_alt(self, use):
        self.using_alt = use


class ADC_PIN(PIN):
    """ADC Pin.

    Class to store details of an ADC-enabled pin.

    """

    def __init__(self, port=None, pin=None, adc_channel=None, enc_channel=None):
        PIN.__init__(self, port, pin, enc_channel)
        self.type.append(PIN_MODE_ADC)
        self.adc_channel = adc_channel


class ADC_OR_PWM_PIN(ADC_PIN, PWM_PIN):
    """ADC/PWM Pin.

    Class to store details of an ADC/PWM-enabled pin.

    """

    def __init__(self, port=None, pin=None, adc_channel=None, pwm_piocon=None, pwm_define=None, enc_channel=None):
        ADC_PIN.__init__(self, port, pin, adc_channel, enc_channel)
        PWM_PIN.__init__(self, port, pin, pwm_piocon, pwm_define, enc_channel)


class ADC_OR_DUAL_PWM_PIN(ADC_PIN, DUAL_PWM_PIN):
    """ADC/PWM Pin, with alt

    Class to store details of an ADC/PWM-enabled pin, with alt

    """

    def __init__(self, port=None, pin=None, adc_channel=None, pwm_piocon=None, pwm_define=None, pwm_auxr=None, pwm_alt_define=None, enc_channel=None):
        ADC_PIN.__init__(self, port, pin, adc_channel, enc_channel)
        DUAL_PWM_PIN.__init__(self, port, pin, pwm_piocon, pwm_define, pwm_auxr, pwm_alt_define, enc_channel)


class PinRegs:
    def __init__(self, m1=None, m2=None, p=None, ps=None, int_mask_p=None):
        # The PxM1 and PxM2 registers encode GPIO MODE
        # 0 0 = Quasi-bidirectional
        # 0 1 = Push-pull
        # 1 0 = Input-only (high-impedance)
        # 1 1 = Open-drain
        self.m1 = m1
        self.m2 = m2

        # The Px input register
        self.p = p
        # The PxS Schmitt trigger register
        self.ps = ps
        self.int_mask_p = int_mask_p


class PWMRegs:
    def __init__(self, piocon=None, pwmcon0=None, pwmcon1=None, pwml=None, pwmh=None):
        self.piocon = piocon
        self.pwmcon0 = pwmcon0
        self.pwmcon1 = pwmcon1
        self.pwml = pwml
        self.pwmh = pwmh


class _IO:
    def __init__(
        self,
        i2c_addr,
        interrupt_timeout=1.0,
        interrupt_pin=None,
        interrupt_pull_up=False,
        gpio=None,
        skip_chip_id_check=False,
        perform_reset=False
    ):
        self._i2c_addr = i2c_addr
        self._i2c_dev = SMBus(1)
        self._debug = False
        self._vref = 3.3
        self._adc_enabled = False
        self._timeout = interrupt_timeout
        self._interrupt_pin = interrupt_pin
        self._gpio = gpio
        self._encoder_offset = [0, 0, 0, 0]
        self._encoder_last = [0, 0, 0, 0]

        # Check the chip ID first, before setting up any GPIO
        if not skip_chip_id_check:
            chip_id = self.get_chip_id()
            if chip_id != self._chip_id:
                raise RuntimeError("Chip ID invalid: {:04x} expected: {:04x}.".format(chip_id, self._chip_id))

        # Reset the chip if requested, to put it into a known state
        if perform_reset:
            self.reset()

        # Set up the interrupt pin on the Pi, and enable the chip's output
        if self._interrupt_pin is not None:
            if self._gpio is None:
                import RPi.GPIO as GPIO
                self._gpio = GPIO
            self._gpio.setwarnings(False)
            self._gpio.setmode(GPIO.BCM)
            if interrupt_pull_up:
                self._gpio.setup(self._interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            else:
                self._gpio.setup(self._interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
            self.enable_interrupt_out()

    def i2c_read8(self, reg):
        """Read a single (8bit) register from the device."""
        msg_w = i2c_msg.write(self._i2c_addr, [reg])
        msg_r = i2c_msg.read(self._i2c_addr, 1)
        self._i2c_dev.i2c_rdwr(msg_w, msg_r)

        return list(msg_r)[0]

    def i2c_read12(self, reg_l, reg_h):
        """Read two (4+8bit) registers from the device, as a single read if they are consecutive."""
        if reg_h == reg_l + 1:
            msg_w = i2c_msg.write(self._i2c_addr, [reg_l])
            msg_r = i2c_msg.read(self._i2c_addr, 2)
            self._i2c_dev.i2c_rdwr(msg_w, msg_r)
            return list(msg_r)[0] | (list(msg_r)[1] << 4)
        else:
            return (self.i2c_read8(reg_h) << 4) | self.i2c_read8(reg_l)

    def i2c_read16(self, reg_l, reg_h):
        """Read two (8+8bit) registers from the device, as a single read if they are consecutive."""
        if reg_h == reg_l + 1:
            msg_w = i2c_msg.write(self._i2c_addr, [reg_l])
            msg_r = i2c_msg.read(self._i2c_addr, 2)
            self._i2c_dev.i2c_rdwr(msg_w, msg_r)
            return list(msg_r)[0] | (list(msg_r)[1] << 8)
        else:
            return self.i2c_read8(reg_l) | (self.i2c_read8(reg_h) << 8)

    def i2c_write8(self, reg, value):
        """Write a single (8bit) register to the device."""
        msg_w = i2c_msg.write(self._i2c_addr, [reg, value])
        self._i2c_dev.i2c_rdwr(msg_w)

    def i2c_write16(self, reg_l, reg_h, value):
        """Write two (8+8bit) registers to the device, as a single write if they are consecutive."""
        val_l = value & 0xff
        val_h = (value >> 8) & 0xff
        if reg_h == reg_l + 1:
            msg_w = i2c_msg.write(self._i2c_addr, [reg_l, val_l, val_h])
            self._i2c_dev.i2c_rdwr(msg_w)
        else:
            msg_wl = i2c_msg.write(self._i2c_addr, [reg_l, val_l])
            msg_wh = i2c_msg.write(self._i2c_addr, [reg_h, val_h])
            self._i2c_dev.i2c_rdwr(msg_wl, msg_wh)

    def get_pin(self, pin):
        """Get a pin definition from its index."""
        if pin < 1 or pin > len(self._pins):
            raise ValueError("Pin should be in range 1-{}.".format(len(self._pins)))

        return self._pins[pin - 1]

    def setup_switch_counter(self, pin, mode=IN_PU):
        """Enable switch counting on a pin."""
        io_pin = self.get_pin(pin)

        if io_pin.port not in (0, 1):
            raise ValueError("Pin {} does not support switch counting.".format(pin))

        if mode not in [IN, IN_PU]:
            raise ValueError("Pin mode should be one of IN or IN_PU")

        self.set_mode(pin, mode, schmitt_trigger=True)

        sw_reg = [self.REG_SWITCH_EN_P0, self.REG_SWITCH_EN_P1][io_pin.port]
        self.set_bit(sw_reg, io_pin.pin)

    def read_switch_counter(self, pin):
        """Read the switch count value on a pin."""
        io_pin = self.get_pin(pin)

        if io_pin.port not in (0, 1):
            raise ValueError("Pin {} does not support switch counting.".format(pin))

        sw_reg = [self.REG_SWITCH_P00, self.REG_SWITCH_P10][io_pin.port] + io_pin.pin

        value = self.i2c_read8(sw_reg)

        # The switch counter is 7-bit
        # The most significant bit encodes the current GPIO state
        return value & 0x7f, value & 0x80 == 0x80

    def clear_switch_counter(self, pin):
        """Clear the switch count value on a pin to 0."""
        io_pin = self.get_pin(pin)

        if io_pin.port not in (0, 1):
            raise ValueError("Pin {} does not support switch counting.".format(pin))

        sw_reg = [self.REG_SWITCH_P00, self.REG_SWITCH_P10][io_pin.port] + io_pin.pin

        self.i2c_write8(sw_reg, 0)

    def setup_rotary_encoder(self, channel, pin_a, pin_b, pin_c=None, count_microsteps=False):
        """Set up a rotary encoder."""
        if channel < 1 or channel > 4:
            raise ValueError("Channel should be in range 1-4.")
        channel -= 1

        enc_channel_a = self.get_pin(pin_a).enc_channel
        enc_channel_b = self.get_pin(pin_b).enc_channel
        if enc_channel_a is None:
            raise ValueError("Pin {} does not support an encoder.".format(pin_a))
        if enc_channel_b is None:
            raise ValueError("Pin {} does not support an encoder.".format(pin_b))

        self.set_mode(pin_a, PIN_MODE_PU, schmitt_trigger=True)
        self.set_mode(pin_b, PIN_MODE_PU, schmitt_trigger=True)
        if pin_c is not None:
            if pin_c < 1 or pin_c > len(self._pins):
                raise ValueError("Pin C should be in range 1-{}, or None.".format(len(self._pins)))
            self.set_mode(pin_c, PIN_MODE_OD)
            self.output(pin_c, 0)

        self.i2c_write8(
            [self.REG_ENC_1_CFG, self.REG_ENC_2_CFG, self.REG_ENC_3_CFG, self.REG_ENC_4_CFG][channel],
            enc_channel_a | (enc_channel_b << 4),
        )
        self.change_bit(self.REG_ENC_EN, channel * 2 + 1, count_microsteps)
        self.set_bit(self.REG_ENC_EN, channel * 2)

        # Reset internal encoder count to zero
        self.clear_rotary_encoder(channel + 1)

    def set_bits(self, reg, bits):
        """Set the specified bits (using a mask) in a register."""
        if reg in self.BIT_ADDRESSED_REGS:
            for bit in range(8):
                if bits & (1 << bit):
                    self.i2c_write8(reg, 0b1000 | (bit & 0b111))
        else:
            value = self.i2c_read8(reg)
            # time.sleep(0.001)
            self.i2c_write8(reg, value | bits)

    def set_bit(self, reg, bit):
        """Set the specified bit (nth position from right) in a register."""
        self.set_bits(reg, (1 << bit))

    def clr_bits(self, reg, bits):
        """Clear the specified bits (using a mask) in a register."""
        if reg in self.BIT_ADDRESSED_REGS:
            for bit in range(8):
                if bits & (1 << bit):
                    self.i2c_write8(reg, 0b0000 | (bit & 0b111))
        else:
            value = self.i2c_read8(reg)
            # time.sleep(0.001)
            self.i2c_write8(reg, value & ~bits)

    def clr_bit(self, reg, bit):
        """Clear the specified bit (nth position from right) in a register."""
        self.clr_bits(reg, (1 << bit))

    def get_bit(self, reg, bit):
        """Returns the specified bit (nth position from right) from a register."""
        return self.i2c_read8(reg) & (1 << bit)

    def change_bit(self, reg, bit, state):
        """Toggle one register bit on/off."""
        if state:
            self.set_bit(reg, bit)
        else:
            self.clr_bit(reg, bit)

    def enable_interrupt_out(self, pin_swap=False):
        """Enable the IOE interrupts."""
        self.set_bit(self.REG_INT, self.BIT_INT_OUT_EN)
        self.change_bit(self.REG_INT, self.BIT_INT_PIN_SWAP, pin_swap)

    def disable_interrupt_out(self):
        """Disable the IOE interrupt output."""
        self.clr_bit(self.REG_INT, self.BIT_INT_OUT_EN)

    def get_interrupt(self):
        """Get the IOE interrupt state."""
        if self._interrupt_pin is not None:
            return self._gpio.input(self._interrupt_pin) == 0
        else:
            return self.get_bit(self.REG_INT, self.BIT_INT_TRIGD)

    def clear_interrupt(self):
        """Clear the interrupt flag."""
        self.clr_bit(self.REG_INT, self.BIT_INT_TRIGD)

    def set_pin_interrupt(self, pin, enabled):
        """Enable/disable the input interrupt on a specific pin.

        :param pin: Pin from 1-14
        :param enabled: True/False for enabled/disabled

        """
        io_pin = self.get_pin(pin)

        self.change_bit(self.get_pin_regs(io_pin).int_mask_p, io_pin.pin, enabled)

    def on_interrupt(self, callback):
        """Attach an event handler to be run on interrupt.

        :param callback: Callback function to run: callback(pin)

        """
        if self._interrupt_pin is not None:
            self._gpio.add_event_detect(self._interrupt_pin, self._gpio.FALLING, callback=callback, bouncetime=1)

    def _wait_for_flash(self):
        """Wait for the IOE to finish writing non-volatile memory."""
        t_start = time.time()
        while self.get_interrupt():
            if time.time() - t_start > self._timeout:
                raise RuntimeError("Timed out waiting for interrupt!")
            time.sleep(0.001)

        t_start = time.time()
        while not self.get_interrupt():
            if time.time() - t_start > self._timeout:
                raise RuntimeError("Timed out waiting for interrupt!")
            time.sleep(0.001)

    def set_i2c_addr(self, i2c_addr):
        """Set the IOE i2c address."""
        self.set_bit(self.REG_CTRL, 4)
        self.i2c_write8(self.REG_ADDR, i2c_addr)
        self._i2c_addr = i2c_addr
        time.sleep(0.25)  # TODO Handle addr change IOError better
        # self._wait_for_flash()
        self.clr_bit(self.REG_CTRL, 4)

    def set_adc_vref(self, vref):
        """Set the ADC voltage reference."""
        self._vref = vref

    def get_adc_vref(self):
        """Get the ADC voltage reference."""
        return self._vref

    def enable_adc(self):
        """Enable the analog to digital converter."""
        self.set_bit(self.REG_ADCCON1, 0)
        self._adc_enabled = True

    def disable_adc(void):
        """Disable the analog to digital converter."""
        self.clr_bit(self.REG_ADCCON1, 0)
        if self.REG_AINDIDS1 is not None:
            self.i2c_write16(self.REG_AINDIDS0, self.REG_AINDIDS1, 0)
        else:
            self.i2c_write8(self.REG_AINDIDS0, 0)
        self._adc_enabled = False

    def get_chip_id(self):
        """Get the IOE chip ID."""
        return self.i2c_read16(self.REG_CHIP_ID_L, self.REG_CHIP_ID_H)

    def get_version(self):
        """Get the IOE version."""
        return self.i2c_read8(self.REG_VERSION)

    def _check_reset(self):
        try:
            return self.i2c_read8(self.REG_USER_FLASH)
        except OSError:
            return 0x00

    def reset(self):
        t_start = time.time()
        self.set_bits(self.REG_CTRL, self.MASK_CTRL_RESET)
        # Wait for a register to read its initialised value
        while self._check_reset() != 0x78:
            time.sleep(0.001)
            if time.time() - t_start >= self._timeout:
                raise RuntimeError("Timed out waiting for Reset!")

    def get_pwm_module(self, pin):
        if pin < 1 or pin > len(self._pins):
            raise ValueError("Pin should be in range 1-{}.".format(len(self._pins)))

        io_pin = self._pins[pin - 1]
        if PIN_MODE_PWM not in io_pin.type:
            io_mode = (PIN_MODE_PWM >> 2) & 0b11
            raise ValueError("Pin {} does not support {}!".format(pin, MODE_NAMES[io_mode]))

        if isinstance(io_pin, DUAL_PWM_PIN) and io_pin.is_using_alt():
            if io_pin.is_using_alt():
                return io_pin.pwm_alt_module
        return io_pin.pwm_module

    def pwm_load(self, pwm_module=0, wait_for_load=True):
        # Load new period and duty registers into buffer
        t_start = time.time()
        self.set_bit(self._regs_pwmcon0[pwm_module], 6)  # Set the "LOAD" bit of PWMCON0
        if wait_for_load:
            while self.pwm_loading(pwm_module):
                time.sleep(0.001)  # Wait for "LOAD" to complete
                if time.time() - t_start >= self._timeout:
                    raise RuntimeError("Timed out waiting for PWM load!")

    def pwm_loading(self, pwm_module=0):
        return self.get_bit(self._regs_pwmcon0[pwm_module], 6)

    def pwm_clear(self, pwm_module=0, wait_for_clear=True):
        # Clear the PWM counter
        t_start = time.time()
        self.set_bit(self._regs_pwmcon0[pwm_module], 4)  # Set the "CLRPWM" bit of PWMCON0
        if wait_for_clear:
            while self.pwm_clearing(pwm_module):
                time.sleep(0.001)  # Wait for "LOAD" to complete
                if time.time() - t_start >= self._timeout:
                    raise RuntimeError("Timed out waiting for PWM clear!")

    def pwm_clearing(self, pwm_module=0):
        return self.get_bit(self._regs_pwmcon0[pwm_module], 4)

    def set_pwm_control(self, divider, pwm_module=0):
        """Set PWM settings.

        PWM is driven by the 24MHz FSYS clock by default.

        :param divider: Clock divider, one of 1, 2, 4, 8, 16, 32, 64 or 128

        """
        try:
            pwmdiv2 = {
                1: 0b000,
                2: 0b001,
                4: 0b010,
                8: 0b011,
                16: 0b100,
                32: 0b101,
                64: 0b110,
                128: 0b111,
            }[divider]
        except KeyError:
            raise ValueError("A clock divider of {}".format(divider))

        # TODO: This currently sets GP, PWMTYP and FBINEN to 0
        # It might be desirable to make these available to the user
        # GP - Group mode enable (changes first three pairs of pAM to PWM01H and PWM01L)
        # PWMTYP - PWM type select: 0 edge-aligned, 1 center-aligned
        # FBINEN - Fault-break input enable

        pwmcon1 = self._regs_pwmcon1[pwm_module]
        self.i2c_write8(pwmcon1, pwmdiv2)

    def set_pwm_period(self, value, pwm_module=0, load=True, wait_for_load=True):
        """Set the PWM period.

        The period is the point at which the PWM counter is reset to zero.

        The PWM clock runs at FSYS with a divider of 1/1.

        Also specifies the maximum value that can be set in the PWM duty cycle.

        """
        pwmpl = self._regs_pwmpl[pwm_module]
        pwmph = self._regs_pwmph[pwm_module]
        self.i2c_write16(pwmpl, pwmph, value)

        # Commented out, as it gets set when the pin is configured
        # pwmcon0 = self._regs_pwmcon0[pwm_module]
        # self.set_bit(pwmcon0, 7)  # Set PWMRUN bit

        if load:
            self.pwm_load(pwm_module, wait_for_load)

    def set_pwm_frequency(self, frequency, pwm_module=0, load=True, wait_for_load=True):
        period = int(CLOCK_FREQ // frequency)
        if period // 128 > MAX_PERIOD:
            raise ValueError("The provided frequency is too low")
        if period < 2:
            raise ValueError("The provided frequency is too high")

        divider = 1

        while (period > MAX_PERIOD) and (divider < MAX_DIVIDER):
            period = period >> 1
            divider = divider << 1

        period = min(period, MAX_PERIOD)  # Should be unnecessary because of earlier raised errors, but kept in case
        self.set_pwm_control(divider, pwm_module)
        self.set_pwm_period(period - 1, pwm_module, load, wait_for_load)

        return period

    def get_mode(self, pin):
        """Get the current mode of a pin."""
        return self._pins[pin - 1].mode

    def set_mode(self, pin, mode, schmitt_trigger=False, invert=False):
        """Set a pin output mode.

        :param mode: one of the supplied IN, OUT, PWM or ADC constants

        """
        io_pin = self.get_pin(pin)
        if io_pin.mode == mode:
            return

        gpio_mode = mode & 0b11
        io_mode = (mode >> 2) & 0b11
        initial_state = mode >> 4

        if io_mode != PIN_MODE_IO and mode not in io_pin.type:
            raise ValueError("Pin {} does not support {}!".format(pin, MODE_NAMES[io_mode]))

        io_pin.mode = mode
        if self._debug:
            print(
                "Setting pin {pin} to mode {mode} {name}, state: {state}".format(
                    pin=pin,
                    mode=MODE_NAMES[io_mode],
                    name=GPIO_NAMES[gpio_mode],
                    state=STATE_NAMES[initial_state],
                )
            )

        if mode == PIN_MODE_PWM:
            self.set_bit(self.get_pwm_regs(io_pin).piocon, io_pin.bit_iopwm)
            if isinstance(io_pin, DUAL_PWM_PIN) and io_pin.is_using_alt():
                if io_pin.pwm_module == 0:  # Only module 0's outputs can be inverted
                    self.change_bit(self.REG_PNP, io_pin.bit_iopwm, invert)
                self.set_bit(self.get_alt_pwm_regs(io_pin).pwmcon0, 7)  # Set PWMRUN bit
            else:
                if io_pin.pwm_module == 0:  # Only module 0's outputs can be inverted
                    self.change_bit(self.REG_PNP, io_pin.bit_iopwm, invert)
                self.set_bit(self.get_pwm_regs(io_pin).pwmcon0, 7)  # Set PWMRUN bit

        elif mode == PIN_MODE_ADC:
            self.enable_adc()

        else:
            if PIN_MODE_PWM in io_pin.type:
                self.clr_bit(self.get_pwm_regs(io_pin).piocon, io_pin.bit_iopwm)

        pm1 = self.i2c_read8(self.get_pin_regs(io_pin).m1)
        pm2 = self.i2c_read8(self.get_pin_regs(io_pin).m2)

        # Clear the pm1 and pm2 bits
        pm1 &= 255 - (1 << io_pin.pin)
        pm2 &= 255 - (1 << io_pin.pin)

        # Set the new pm1 and pm2 bits according to our gpio_mode
        pm1 |= (gpio_mode >> 1) << io_pin.pin
        pm2 |= (gpio_mode & 0b1) << io_pin.pin

        self.i2c_write8(self.get_pin_regs(io_pin).m1, pm1)
        self.i2c_write8(self.get_pin_regs(io_pin).m2, pm2)

        # Set up Schmitt trigger mode on inputs
        if mode in [PIN_MODE_PU, PIN_MODE_IN]:
            self.change_bit(self.get_pin_regs(io_pin).ps, io_pin.pin, schmitt_trigger)

        # If pin is a bsic output, invert its initial state
        if mode == PIN_MODE_PP and invert:
            initial_state = not initial_state
            io_pin.set_inverted(True)
        else:
            io_pin.set_inverted(False)

        # 5th bit of mode encodes default output pin state
        self.i2c_write8(self.get_pin_regs(io_pin).p, (initial_state << 3) | io_pin.pin)

    def input(self, pin, adc_timeout=1):
        """Read the IO pin state.

        Returns a 12-bit ADC reading if the pin is in ADC mode
        Returns True/False if the pin is in any other input mode
        Returns None if the pin is in PWM mode

        :param adc_timeout: Timeout (in seconds) for an ADC read (default 1.0)

        """
        io_pin = self.get_pin(pin)

        if io_pin.mode == PIN_MODE_ADC:
            if self._debug:
                print("Reading ADC from pin {}".format(pin))

            if io_pin.adc_channel > 8:
                self.i2c_write8(self.REG_AINDIDS1, 1 << (io_pin.adc_channel - 8))
            else:
                self.i2c_write8(self.REG_AINDIDS0, 1 << io_pin.adc_channel)

            con0value = self.i2c_read8(self.REG_ADCCON0)
            con0value = con0value & ~0x0f
            con0value = con0value | io_pin.adc_channel

            con0value = con0value & ~(1 << 7)   # ADCF - Clear the conversion complete flag
            con0value = con0value | (1 << 6)    # ADCS - Set the ADC conversion start flag
            self.i2c_write8(self.REG_ADCCON0, con0value)

            if adc_timeout is not None:
                # Wait for the ADCF conversion complete flag to be set
                t_start = time.time()
                while not self.get_bit(self.REG_ADCCON0, 7):
                    time.sleep(0.001)
                    if time.time() - t_start >= adc_timeout:
                        raise RuntimeError("Timeout waiting for ADC conversion!")

            reading = self.i2c_read12(self.REG_ADCRL, self.REG_ADCRH)
            return (reading / 4095.0) * self._vref
        else:
            if self._debug:
                print("Reading IO from pin {}".format(pin))
            pv = self.get_bit(self.get_pin_regs(io_pin).p, io_pin.pin)

            return HIGH if pv else LOW

    def output(self, pin, value, load=True, wait_for_load=True):
        """Write an IO pin state or PWM duty cycle.

        :param value: Either True/False for OUT, or a number between 0 and PWM period for PWM.

        """
        io_pin = self.get_pin(pin)

        if io_pin.mode == PIN_MODE_PWM:
            if self._debug:
                print("Outputting PWM to pin: {pin}".format(pin=pin))

            if isinstance(io_pin, DUAL_PWM_PIN) and io_pin.is_using_alt():
                alt_regs = self.get_alt_pwm_regs(io_pin)
                self.i2c_write16(alt_regs.pwml, alt_regs.pwmh, value)
                if load:
                    self.pwm_load(io_pin.pwm_alt_module, wait_for_load)
            else:
                regs = self.get_pwm_regs(io_pin)
                self.i2c_write16(regs.pwml, regs.pwmh, value)
                if load:
                    self.pwm_load(io_pin.pwm_module, wait_for_load)
        else:
            if value == LOW:
                if self._debug:
                    print("Outputting LOW to pin: {pin} (or HIGH if inverted)".format(pin=pin))
                self.change_bit(self.get_pin_regs(io_pin).p, io_pin.pin, io_pin.is_inverted())
            elif value == HIGH:
                if self._debug:
                    print("Outputting HIGH to pin: {pin} (or LOW if inverted)".format(pin=pin))
                self.change_bit(self.get_pin_regs(io_pin).p, io_pin.pin, not io_pin.is_inverted())

    def get_pwm_regs(self, pin):
        return PWMRegs(
            piocon=self._regs_piocon[pin.reg_iopwm],
            pwmcon0=self._regs_pwmcon0[pin.pwm_module],
            pwmcon1=self._regs_pwmcon1[pin.pwm_module],
            pwml=self._regs_pwml[pin.pwm_module][pin.pwm_channel],
            pwmh=self._regs_pwmh[pin.pwm_module][pin.pwm_channel],
        )

    def get_alt_pwm_regs(self, pin):
        return PWMRegs(
            piocon=self._regs_piocon[pin.reg_iopwm],
            pwmcon0=self._regs_pwmcon0[pin.pwm_alt_module],
            pwmcon1=self._regs_pwmcon1[pin.pwm_alt_module],
            pwml=self._regs_pwml[pin.pwm_alt_module][pin.pwm_alt_channel],
            pwmh=self._regs_pwmh[pin.pwm_alt_module][pin.pwm_alt_channel],
        )

    def get_pin_regs(self, pin):
        return PinRegs(
            m1=self._regs_m1[pin.port],
            m2=self._regs_m2[pin.port],
            p=self._regs_p[pin.port],
            ps=self._regs_ps[pin.port],
            int_mask_p=self._regs_int_mask_p[pin.port],
        )

    def switch_pwm_to_alt(self, pin):
        if pin < 1 or pin > len(self._pins):
            raise ValueError("Pin should be in range 1-{}.".format(len(self._pins)))

        io_pin = self._pins[pin - 1]

        if not isinstance(io_pin, DUAL_PWM_PIN):
            raise ValueError("Pin does not have an alternate PWM.")

        auxr = self.i2c_read8(self._regs_auxr[io_pin.reg_auxr])
        auxr = auxr & ~(0b11 << io_pin.bit_auxr)            # Clear the bits for the alt output
        auxr = auxr | (io_pin.val_auxr << io_pin.bit_auxr)  # Set the bits for outputting the aux to the intended pin

        self.i2c_write8(self._regs_auxr[io_pin.reg_auxr], auxr)
        io_pin.set_using_alt(True)


class IOE(_IO, ioe_regs.REGS):
    def __init__(
        self,
        i2c_addr=None,
        interrupt_timeout=1.0,
        interrupt_pin=None,
        interrupt_pull_up=False,
        gpio=None,
        skip_chip_id_check=False,
        perform_reset=False
    ):
        self._pins = [                                                                                            # Pin |  ADC   |  PWM   |  ENC  |
            PWM_PIN(port=1, pin=5, pwm_piocon=(1, 5), pwm_define=(0, 5), enc_channel=1),                          # 1   |        | [CH 5] | CH 1  |
            PWM_PIN(port=1, pin=0, pwm_piocon=(0, 2), pwm_define=(0, 2), enc_channel=2),                          # 2   |        | [CH 2] | CH 2  |
            PWM_PIN(port=1, pin=2, pwm_piocon=(0, 0), pwm_define=(0, 0), enc_channel=3),                          # 3   |        | [CH 0] | CH 3  |
            PWM_PIN(port=1, pin=4, pwm_piocon=(1, 1), pwm_define=(0, 1), enc_channel=4),                          # 4   |        | [CH 1] | CH 4  |
            PWM_PIN(port=0, pin=0, pwm_piocon=(0, 3), pwm_define=(0, 3), enc_channel=5),                          # 5   |        | [CH 3] | CH 5  |
            PWM_PIN(port=0, pin=1, pwm_piocon=(0, 4), pwm_define=(0, 4), enc_channel=6),                          # 6   |        | [CH 4] | CH 6  |
            ADC_OR_PWM_PIN(port=1, pin=1, adc_channel=7, pwm_piocon=(0, 1), pwm_define=(0, 1), enc_channel=7),    # 7   | [CH 7] |  CH 1  | CH 7  |
            ADC_OR_PWM_PIN(port=0, pin=3, adc_channel=6, pwm_piocon=(0, 5), pwm_define=(0, 5), enc_channel=8),    # 8   | [CH 6] |  CH 5  | CH 8  |
            ADC_OR_PWM_PIN(port=0, pin=4, adc_channel=5, pwm_piocon=(1, 3), pwm_define=(0, 3), enc_channel=9),    # 9   | [CH 5] |  CH 3  | CH 9  |
            ADC_PIN(port=3, pin=0, adc_channel=1, enc_channel=10),                                                # 10  | [CH 1] |        | CH 10 |
            ADC_PIN(port=0, pin=6, adc_channel=3, enc_channel=11),                                                # 11  | [CH 3] |        | CH 11 |
            ADC_OR_PWM_PIN(port=0, pin=5, adc_channel=4, pwm_piocon=(1, 2), pwm_define=(0, 2), enc_channel=12),   # 12  | [CH 4] |  CH 2  | CH 12 |
            ADC_PIN(port=0, pin=7, adc_channel=2, enc_channel=13),                                                # 13  | [CH 2] |        | CH 13 |
            ADC_PIN(port=1, pin=7, adc_channel=0, enc_channel=14),                                                # 14  | [CH 0] |        | CH 14 |
        ]                                                                                                         # [] = labelled pin functions

        self._regs_m1 = [self.REG_P0M1, self.REG_P1M1, -1, self.REG_P3M1]
        self._regs_m2 = [self.REG_P0M2, self.REG_P1M2, -1, self.REG_P3M2]
        self._regs_p = [self.REG_P0, self.REG_P1, self.REG_P2, self.REG_P3]
        self._regs_ps = [self.REG_P0S, self.REG_P1S, self.REG_P2S, self.REG_P3S]
        self._regs_int_mask_p = [self.REG_INT_MASK_P0, self.REG_INT_MASK_P1, -1, self.REG_INT_MASK_P3]

        self._regs_piocon = [self.REG_PIOCON0, self.REG_PIOCON1]
        self._regs_auxr = [-1, self.REG_AUXR1]

        self._regs_pwmcon0 = [self.REG_PWMCON0]
        self._regs_pwmcon1 = [self.REG_PWMCON1]

        self._regs_pwmpl = [self.REG_PWMPL]
        self._regs_pwmph = [self.REG_PWMPH]

        self._regs_pwml = [
            [self.REG_PWM0L, self.REG_PWM1L, self.REG_PWM2L, self.REG_PWM3L, self.REG_PWM4L, self.REG_PWM5L],
        ]
        self._regs_pwmh = [
            [self.REG_PWM0H, self.REG_PWM1H, self.REG_PWM2H, self.REG_PWM3H, self.REG_PWM4H, self.REG_PWM5H],
        ]

        self._chip_id = self.CHIP_ID

        if i2c_addr is None:
            i2c_addr = self.I2C_ADDR

        _IO.__init__(self, i2c_addr, interrupt_timeout, interrupt_pin, interrupt_pull_up, gpio, skip_chip_id_check, perform_reset)

    def read_rotary_encoder(self, channel):
        """Read the step count from a rotary encoder."""
        if channel < 1 or channel > 4:
            raise ValueError("Channel should be in range 1-4.")
        channel -= 1
        last = self._encoder_last[channel]
        reg = [self.REG_ENC_1_COUNT, self.REG_ENC_2_COUNT, self.REG_ENC_3_COUNT, self.REG_ENC_4_COUNT][channel]
        value = self.i2c_read8(reg)

        if value & 0b10000000:
            value -= 256

        if last > 64 and value < -64:
            self._encoder_offset[channel] += 256
        if last < -64 and value > 64:
            self._encoder_offset[channel] -= 256

        self._encoder_last[channel] = value

        return self._encoder_offset[channel] + value

    def clear_rotary_encoder(self, channel):
        """Clear the rotary encoder count value on a channel to 0."""
        if channel < 1 or channel > 4:
            raise ValueError("Channel should be in range 1-4.")
        channel -= 1

        # Reset internal encoder count to zero
        reg = [self.REG_ENC_1_COUNT, self.REG_ENC_2_COUNT, self.REG_ENC_3_COUNT, self.REG_ENC_4_COUNT][channel]
        self.i2c_write8(reg, 0)
        self._encoder_last[channel] = 0
        self._encoder_offset[channel] = 0


class SuperIOE(_IO, sioe_regs.REGS):
    def __init__(
        self,
        i2c_addr=None,
        interrupt_timeout=1.0,
        interrupt_pin=None,
        interrupt_pull_up=False,
        gpio=None,
        skip_chip_id_check=False,
        perform_reset=False,
        is_super_io=True
    ):
        self._pins = [                                                                                                                                              # Pin |   ADC   |     PWM      |   ALT PWM    |  ENC  |
            PIN(port=3, pin=5, enc_channel=14),                                                                                                                     # [1] |         |              |              | CH 14 |
            PIN(port=3, pin=6, enc_channel=15),                                                                                                                     # [2] |         |              |              | CH 15 |
            ADC_PIN(port=0, pin=6, adc_channel=3, enc_channel=11),                                                                                                  # 3   | [CH 3]  |              |              | CH 11 |
            ADC_PIN(port=0, pin=7, adc_channel=2, enc_channel=13),                                                                                                  # 4   | [CH 2]  |              |              | CH 13 |
            ADC_OR_PWM_PIN(port=1, pin=7, adc_channel=0, pwm_piocon=(1, 7), pwm_define=(3, 0)),                                                                     # 5   | [CH 0]  |  MOD 3 CH 0  |              |       |
            ADC_OR_PWM_PIN(port=3, pin=0, adc_channel=1, pwm_piocon=(2, 4), pwm_define=(2, 1), enc_channel=10),                                                     # 6   | [CH 1]  |  MOD 2 CH 1  |              | CH 10 |
            ADC_OR_DUAL_PWM_PIN(port=0, pin=4, adc_channel=5, pwm_piocon=(1, 3), pwm_define=(0, 3), pwm_auxr=(4, 6, 0b11), pwm_alt_define=(2, 1), enc_channel=9),   # 7   | [CH 5]  |  MOD 0 CH 3  |  MOD 2 CH 1  | CH 9  |
            ADC_OR_DUAL_PWM_PIN(port=0, pin=5, adc_channel=4, pwm_piocon=(1, 2), pwm_define=(0, 2), pwm_auxr=(4, 4, 0b11), pwm_alt_define=(2, 0)),                  # 8   | [CH 4]  |  MOD 0 CH 2  |  MOD 2 CH 0  |       |
            ADC_PIN(port=1, pin=3, adc_channel=13, enc_channel=0),                                                                                                  # 9   | [CH 13] |              |              | CH 0  |
            ADC_PIN(port=2, pin=5, adc_channel=15),                                                                                                                 # 10  | [CH 15] |              |              |       |
            ADC_OR_DUAL_PWM_PIN(port=1, pin=1, adc_channel=7, pwm_piocon=(0, 1), pwm_define=(0, 1), pwm_auxr=(4, 2, 0b11), pwm_alt_define=(1, 1)),                  # 11  | [CH 7]  |  MOD 0 CH 1  |  MOD 1 CH 1  |       |
            ADC_OR_DUAL_PWM_PIN(port=0, pin=3, adc_channel=6, pwm_piocon=(0, 5), pwm_define=(0, 5), pwm_auxr=(5, 6, 0b11), pwm_alt_define=(3, 1), enc_channel=8),   # 12  | [CH 6]  |  MOD 0 CH 5  |  MOD 3 CH 1  | CH 8  |
            ADC_PIN(port=2, pin=4, adc_channel=12, enc_channel=7),                                                                                                  # 13  | [CH 12] |              |              | CH 7  |
            ADC_OR_PWM_PIN(port=2, pin=3, adc_channel=11, pwm_piocon=(2, 2), pwm_define=(1, 0), enc_channel=6),                                                     # 14  | [CH 11] |  MOD 1 CH 0  |              | CH 6  |
            PWM_PIN(port=3, pin=3, pwm_piocon=(2, 6), pwm_define=(0, 0)),                                                                                           # 15  |         | [MOD 0 CH 0] |              |       |
            DUAL_PWM_PIN(port=0, pin=1, pwm_piocon=(0, 4), pwm_define=(0, 4), pwm_auxr=(5, 4, 0b10), pwm_alt_define=(3, 0)),                                        # 16  |         | [MOD 0 CH 4] |  MOD 3 CH 0  |       |
            DUAL_PWM_PIN(port=1, pin=5, pwm_piocon=(1, 5), pwm_define=(0, 5), pwm_auxr=(5, 2, 0b10), pwm_alt_define=(3, 1), enc_channel=1),                         # 17  |         | [MOD 0 CH 5] |  MOD 3 CH 1  | CH 1  |
            ADC_OR_DUAL_PWM_PIN(port=1, pin=4, adc_channel=14, pwm_piocon=(1, 1), pwm_define=(0, 1), pwm_auxr=(4, 2, 0b10), pwm_alt_define=(1, 1), enc_channel=4),  # 18  |  CH 14  | [MOD 0 CH 1] |  MOD 1 CH 1  | CH 4  |
            DUAL_PWM_PIN(port=0, pin=0, pwm_piocon=(0, 3), pwm_define=(0, 3), pwm_auxr=(4, 6, 0b10), pwm_alt_define=(2, 1), enc_channel=5),                         # 19  |         | [MOD 0 CH 3] |  MOD 2 CH 1  | CH 5  |
            DUAL_PWM_PIN(port=1, pin=0, pwm_piocon=(0, 2), pwm_define=(0, 2), pwm_auxr=(4, 4, 0b10), pwm_alt_define=(2, 0), enc_channel=2),                         # 20  |         | [MOD 0 CH 2] |  MOD 2 CH 0  | CH 2  |
            ADC_OR_PWM_PIN(port=2, pin=1, adc_channel=9, pwm_piocon=(2, 0), pwm_define=(2, 0)),                                                                     # 21  |  CH 9   | [MOD 2 CH 0] |              |       |
            ADC_OR_PWM_PIN(port=2, pin=2, adc_channel=10, pwm_piocon=(2, 1), pwm_define=(1, 1)),                                                                    # 22  |  CH 10  | [MOD 1 CH 1] |              |       |
            DUAL_PWM_PIN(port=1, pin=2, pwm_piocon=(0, 0), pwm_define=(0, 0), pwm_auxr=(4, 0, 0b10), pwm_alt_define=(1, 0), enc_channel=3),                         # 23  |         |  MOD 0 CH 0  | [MOD 1 CH 0] | CH 3  |
            PWM_PIN(port=3, pin=2, pwm_piocon=(2, 5), pwm_define=(3, 0)),                                                                                           # 24  |         | [MOD 3 CH 0] |              |       |
            PWM_PIN(port=3, pin=4, pwm_piocon=(2, 7), pwm_define=(3, 1)),                                                                                           # 25  |         | [MOD 3 CH 1] |              |       |
            PWM_PIN(port=3, pin=1, pwm_piocon=(2, 4), pwm_define=(2, 1), enc_channel=12),                                                                           # 26  |         | [MOD 2 CH 1] |              | CH 12 |
        ]                                                                                                                                                           # [] = labelled pin functions

        self._regs_m1 = [self.REG_P0M1, self.REG_P1M1, self.REG_P2M1, self.REG_P3M1]
        self._regs_m2 = [self.REG_P0M2, self.REG_P1M2, self.REG_P2M2, self.REG_P3M2]
        self._regs_p = [self.REG_P0, self.REG_P1, self.REG_P2, self.REG_P3]
        self._regs_ps = [self.REG_P0S, self.REG_P1S, self.REG_P2S, self.REG_P3S]
        self._regs_int_mask_p = [self.REG_INT_MASK_P0, self.REG_INT_MASK_P1, self.REG_INT_MASK_P2, self.REG_INT_MASK_P3]

        self._regs_piocon = [self.REG_PIOCON0, self.REG_PIOCON1, self.REG_PIOCON2]
        self._regs_auxr = [-1, self.REG_AUXR1, -1, -1, self.REG_AUXR4, self.REG_AUXR5, -1, self.REG_AUXR7, self.REG_AUXR8]

        self._regs_pwmcon0 = [self.REG_PWM0CON0, self.REG_PWM1CON0, self.REG_PWM2CON0, self.REG_PWM3CON0]
        self._regs_pwmcon1 = [self.REG_PWM0CON1, self.REG_PWM1CON1, self.REG_PWM2CON1, self.REG_PWM3CON1]

        self._regs_pwmpl = [self.REG_PWM0PL, self.REG_PWM1PL, self.REG_PWM2PL, self.REG_PWM3PL]
        self._regs_pwmph = [self.REG_PWM0PH, self.REG_PWM1PH, self.REG_PWM2PH, self.REG_PWM3PH]

        self._regs_pwml = [
            [self.REG_PWM0C0L, self.REG_PWM0C1L, self.REG_PWM0C2L, self.REG_PWM0C3L, self.REG_PWM0C4L, self.REG_PWM0C5L],
            [self.REG_PWM1C0L, self.REG_PWM1C1L],
            [self.REG_PWM2C0L, self.REG_PWM2C1L],
            [self.REG_PWM3C0L, self.REG_PWM3C1L],
        ]
        self._regs_pwmh = [
            [self.REG_PWM0C0H, self.REG_PWM0C1H, self.REG_PWM0C2H, self.REG_PWM0C3H, self.REG_PWM0C4H, self.REG_PWM0C5H],
            [self.REG_PWM1C0H, self.REG_PWM1C1H],
            [self.REG_PWM2C0H, self.REG_PWM2C1H],
            [self.REG_PWM3C0H, self.REG_PWM3C1H],
        ]

        self._chip_id = self.CHIP_ID

        if i2c_addr is None:
            i2c_addr = self.I2C_ADDR

        _IO.__init__(self, i2c_addr, interrupt_timeout, interrupt_pin, interrupt_pull_up, gpio, skip_chip_id_check, perform_reset)

        if is_super_io:
            # Mux p1.2 PWM over to PWM 1 Channel 0
            self.switch_pwm_to_alt(23)

    def activate_watchdog(self):
        self.clear_watchdog_timeout()
        self.set_bit(self.REG_PERIPHERALS, self.BIT_WATCHDOG)  # Activate the Watchdog

    def deactivate_Watchdog(self):
        self.clr_bit(self.REG_PERIPHERALS, self.BIT_WATCHDOG)  # Deactivate the watchdog

    def is_watchdog_active(self):
        return self.get_bit(self.REG_PERIPHERALS, self.BIT_WATCHDOG)

    def reset_watchdog_counter(self):
        self.set_bit(self.REG_WDCON, 6)  # Set the WDCLR bit to reset the counter

    def watchdog_timeout_occurred(self):
        return self.get_bit(self.REG_WDCON, 3)  # Get the WDTRF bit, which flags if a reset has occurred

    def clear_watchdog_timeout(self):
        self.clr_bit(self.REG_WDCON, 3)  # Clear the WDTRF bit that may have been set from a previous watchdog timeout

    def set_watchdog_control(self, divider):
        try:
            wdtdiv = {
                1: 0b000,
                2: 0b001,
                4: 0b010,
                8: 0b011,
                16: 0b100,
                32: 0b101,
                64: 0b110,
                128: 0b111,
            }[divider]
        except KeyError:
            raise ValueError("A clock divider of {}".format(divider))

        wdt = self.i2c_read8(self.REG_WDCON)
        wdt = wdt & 0b11111000  # Clear the WDPS bits
        wdt = wdt | wdtdiv  # Set the new WDPS bits according to our divider

        self.i2c_write8(self.REG_WDCON, wdt)

    def i2c_multi_read(self, reg_base, count):
        """Read two (8bit) register from the device, as a single read if they are consecutive."""
        if count <= 0:
            raise ValueError("Count must be greater than zero!")

        msg_w = i2c_msg.write(self._i2c_addr, [reg_base])
        msg_r = i2c_msg.read(self._i2c_addr, count)
        self._i2c_dev.i2c_rdwr(msg_w, msg_r)
        return list(msg_r)

    def read_rotary_encoder(self, channel):
        """Read the step count from a rotary encoder."""
        if channel < 1 or channel > 4:
            raise ValueError("Channel should be in range 1-4.")
        channel -= 1
        last = self._encoder_last[channel]
        reg = [self.REG_ENC_1_COUNT, self.REG_ENC_2_COUNT, self.REG_ENC_3_COUNT, self.REG_ENC_4_COUNT][channel]
        value = self.i2c_read16(reg, reg + 1)

        if value & 0b1000000000000000:
            value -= 65536

        if last > 16384 and value < -16384:
            self._encoder_offset[channel] += 65536
        if last < -16384 and value > 16384:
            self._encoder_offset[channel] -= 65536

        self._encoder_last[channel] = value

        return self._encoder_offset[channel] + value

    def clear_rotary_encoder(self, channel):
        """Clear the rotary encoder count value on a channel to 0."""
        if channel < 1 or channel > 4:
            raise ValueError("Channel should be in range 1-4.")
        channel -= 1

        # Reset internal encoder count to zero
        reg = [self.REG_ENC_1_COUNT, self.REG_ENC_2_COUNT, self.REG_ENC_3_COUNT, self.REG_ENC_4_COUNT][channel]
        self.i2c_write16(reg, reg + 1, 0)
        self._encoder_last[channel] = 0
        self._encoder_offset[channel] = 0

    def read_rotary_encoders(self, start_channel, end_channel):
        """Read the step count from a group of rotary encoders."""
        if start_channel < 1 or start_channel > 4:
            raise ValueError("Start Channel should be in range 1-4.")

        if end_channel < start_channel or end_channel > 4:
            raise ValueError("End Channel should be in range 1-4, and greater or equal to Start Channel.")
        start_channel -= 1
        # No need to sub 1 from end_channel, as it would only be added back later

        reg = [self.REG_ENC_1_COUNT, self.REG_ENC_2_COUNT, self.REG_ENC_3_COUNT, self.REG_ENC_4_COUNT][start_channel]
        values = self.i2c_multi_read(reg, end_channel - start_channel)

        counts = []
        for channel in range(start_channel, end_channel):
            last = self._encoder_last[channel]
            value = values[channel - start_channel]
            if value & 0b1000000000000000:
                value -= 65536

            if last > 16384 and value < -16384:
                self._encoder_offset[channel] += 65536
            if last < -16384 and value > 16384:
                self._encoder_offset[channel] -= 65536

            self._encoder_last[channel] = value
            counts.append(self._encoder_offset[channel] + value)

        return counts
