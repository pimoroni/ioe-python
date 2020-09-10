import time

from smbus2 import SMBus, i2c_msg

from . import sioe_regs
from . import ioe_regs

__version__ = '0.0.3'


# These values encode our desired pin function: IO, ADC, PWM
# alongwide the GPIO MODE for that port and pin (section 8.1)
# the 5th bit additionally encodes the default output state
PIN_MODE_IO = 0b00000  # General IO mode, IE: not ADC or PWM
PIN_MODE_QB = 0b00000  # Output, Quasi-Bidirectional mode
PIN_MODE_PP = 0b00001  # Output, Push-Pull mode
PIN_MODE_IN = 0b00010  # Input-only (high-impedance)
PIN_MODE_PU = 0b10000  # Input (with pull-up)
PIN_MODE_OD = 0b00011  # Output, Open-Drain mode
PIN_MODE_PWM = 0b00101  # PWM, Output, Push-Pull mode
PIN_MODE_ADC = 0b01010  # ADC, Input-only (high-impedance)
MODE_NAMES = ("IO", "PWM", "ADC")
GPIO_NAMES = ("QB", "PP", "IN", "OD")
STATE_NAMES = ("LOW", "HIGH")

IN = PIN_MODE_IN
IN_PULL_UP = PIN_MODE_PU
IN_PU = PIN_MODE_PU
OUT = PIN_MODE_PP
PWM = PIN_MODE_PWM
ADC = PIN_MODE_ADC

HIGH = 1
LOW = 0


class PIN:
    def __init__(self, port=None, pin=None):
        if getattr(self, "type", None) is None:
            self.type = [PIN_MODE_IO]
        self.mode = None
        self.port = port
        self.pin = pin


class PWM_PIN(PIN):
    """PWM Pin.

    Class to store details of a PWM-enabled pin.

    """

    def __init__(self, port=None, pin=None, pwm_piocon=None, pwm_channel=None):
        PIN.__init__(self, port, pin)
        self.type.append(PIN_MODE_PWM)
        self.pwm_generator, self.pwm_channel = pwm_channel
        self.reg_iopwm, self.bit_iopwm = pwm_piocon


class ADC_PIN(PIN):
    """ADC Pin.

    Class to store details of an ADC-enabled pin.

    """

    def __init__(self, port=None, pin=None, adc_channel=None):
        PIN.__init__(self, port, pin)
        self.type.append(PIN_MODE_ADC)
        self.adc_channel = adc_channel


class ADC_OR_PWM_PIN(ADC_PIN, PWM_PIN):
    """ADC/PWM Pin.

    Class to store details of an ADC/PWM-enabled pin.

    """

    def __init__(self, port=None, pin=None, adc_channel=None, pwm_piocon=None, pwm_channel=None):
        ADC_PIN.__init__(self, port, pin, adc_channel)
        PWM_PIN.__init__(self, port, pin, pwm_piocon, pwm_channel)


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
        gpio=None,
        skip_chip_id_check=False,
    ):
        self._i2c_addr = i2c_addr
        self._i2c_dev = SMBus(1)
        self._debug = False
        self._vref = 3.3
        self._timeout = interrupt_timeout
        self._interrupt_pin = interrupt_pin
        self._gpio = gpio
        self._encoder_offset = [0, 0, 0, 0]
        self._encoder_last = [0, 0, 0, 0]

        if self._interrupt_pin is not None:
            if self._gpio is None:
                import RPi.GPIO as GPIO

                self._gpio = GPIO
            self._gpio.setwarnings(False)
            self._gpio.setmode(GPIO.BCM)
            self._gpio.setup(self._interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
            self.enable_interrupt_out()

        if not skip_chip_id_check:
            chip_id = (self.i2c_read8(self.REG_CHIP_ID_H) << 8) | self.i2c_read8(self.REG_CHIP_ID_L)
            if chip_id != self._chip_id:
                raise RuntimeError("Chip ID invalid: {:04x} expected: {:04x}.".format(chip_id, self._chip_id))

    def i2c_read8(self, reg):
        """Read a single (8bit) register from the device."""
        msg_w = i2c_msg.write(self._i2c_addr, [reg])
        msg_r = i2c_msg.read(self._i2c_addr, 1)
        self._i2c_dev.i2c_rdwr(msg_w, msg_r)

        return list(msg_r)[0]

    def i2c_write8(self, reg, value):
        """Write a single (8bit) register to the device."""
        msg_w = i2c_msg.write(self._i2c_addr, [reg, value])
        self._i2c_dev.i2c_rdwr(msg_w)

    def get_pin(self, pin):
        """Get a pin definition from its index."""
        if pin < 1 or pin > len(self._pins):
            raise ValueError("Pin should be in range 1-14.")

        return self._pins[pin - 1]

    def setup_switch_counter(self, pin, mode=IN_PU):
        """Enable switch counting on a pin."""
        io_pin = self.get_pin(pin)

        if io_pin.port not in (0, 1):
            raise ValueError("Pin {} does not support switch counting.".format(pin))

        if mode not in [IN, IN_PU]:
            raise ValueError("Pin mode should be one of IN or IN_PU")

        self.set_mode(pin, mode, schmitt_trigger=True)

        sw_reg = [REG_SWITCH_EN_P0, REG_SWITCH_EN_P1][io_pin.port]
        self.set_bit(sw_reg, io_pin.pin)

    def read_switch_counter(self, pin):
        """Read the switch count value on a pin."""
        io_pin = self.get_pin(pin)

        if io_pin.port not in (0, 1):
            raise ValueError("Pin {} does not support switch counting.".format(pin))

        sw_reg = [REG_SWITCH_P00, REG_SWITCH_P10][io_pin.port] + io_pin.pin

        value = self.i2c_read8(sw_reg)

        # The switch counter is 7-bit
        # The most significant bit encodes the current GPIO state
        return value & 0x7f, value & 0x80 == 0x80

    def clear_switch_counter(self, pin):
        """Clear the switch count value on a pin to 0."""
        io_pin = self.get_pin(pin)

        if io_pin.port not in (0, 1):
            raise ValueError("Pin {} does not support switch counting.".format(pin))

        sw_reg = [REG_SWITCH_P00, REG_SWITCH_P10][io_pin.port] + io_pin.pin

        self.i2c_write8(sw_reg, 0)

    def setup_rotary_encoder(self, channel, pin_a, pin_b, pin_c=None, count_microsteps=False):
        """Set up a rotary encoder."""
        channel -= 1
        self.set_mode(pin_a, PIN_MODE_PU, schmitt_trigger=True)
        self.set_mode(pin_b, PIN_MODE_PU, schmitt_trigger=True)
        if pin_c is not None:
            self.set_mode(pin_c, PIN_MODE_OD)
            self.output(pin_c, 0)

        self.i2c_write8(
            [
                self.REG_ENC_1_CFG,
                self.REG_ENC_2_CFG,
                self.REG_ENC_3_CFG,
                self.REG_ENC_4_CFG,
            ][channel],
            pin_a | (pin_b << 4),
        )
        self.change_bit(self.REG_ENC_EN, channel * 2 + 1, count_microsteps)
        self.set_bit(self.REG_ENC_EN, channel * 2)

    def read_rotary_encoder(self, channel):
        """Read the step count from a rotary encoder."""
        channel -= 1
        last = self._encoder_last[channel]
        reg = [
            self.REG_ENC_1_COUNT,
            self.REG_ENC_2_COUNT,
            self.REG_ENC_3_COUNT,
            self.REG_ENC_4_COUNT,
        ][channel]
        value = self.i2c_read8(reg)

        if value & 0b10000000:
            value -= 256

        if last > 64 and value < -64:
            self._encoder_offset[channel] += 256
        if last < -64 and value > 64:
            self._encoder_offset[channel] -= 256

        self._encoder_last[channel] = value

        return self._encoder_offset[channel] + value

    def set_bits(self, reg, bits):
        """Set the specified bits (using a mask) in a register."""
        if reg in self.BIT_ADDRESSED_REGS:
            for bit in range(8):
                if bits & (1 << bit):
                    self.i2c_write8(reg, 0b1000 | (bit & 0b111))
        else:
            value = self.i2c_read8(reg)
            time.sleep(0.001)
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
            time.sleep(0.001)
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

    def get_chip_id(self):
        """Get the IOE chip ID."""
        return (self.i2c_read8(self.REG_CHIP_ID_H) << 8) | self.i2c_read8(self.REG_CHIP_ID_L)

    def _pwm_load(self, reg):
        # Load new period and duty registers into buffer
        t_start = time.time()
        self.set_bit(reg, 6)  # Set the "LOAD" bit of PWMCON0
        while self.get_bit(reg, 6):
            time.sleep(0.001)  # Wait for "LOAD" to complete
            if time.time() - t_start >= self._timeout:
                raise RuntimeError("Timed out waiting for PWM load!")

    def set_pwm_control(self, divider, pwm_generator=0):
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

        pwmcon1 = self._regs_pwmcon1[pwm_generator]
        self.i2c_write8(pwmcon1, pwmdiv2)

    def set_pwm_period(self, value, pwm_generator=0):
        """Set the PWM period.

        The period is the point at which the PWM counter is reset to zero.

        The PWM clock runs at FSYS with a divider of 1/1.

        Also specifies the maximum value that can be set in the PWM duty cycle.

        """
        pwmcon0 = self._regs_pwmcon0[pwm_generator]
        pwmpl = self._regs_pwmpl[pwm_generator]
        pwmph = self._regs_pwmph[pwm_generator]

        value &= 0xFFFF
        self.i2c_write8(pwmpl, value & 0xFF)
        self.i2c_write8(pwmph, value >> 8)

        self.set_bit(pwmcon0, 7)  # Set PWMRUN bit
        self._pwm_load(pwmcon0)

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
            if io_pin.pwm_generator == 0:
                self.change_bit(self.REG_PNP, io_pin.bit_iopwm, invert)
            self.set_bit(self.get_pwm_regs(io_pin).pwmcon0, 7)  # Set PWMRUN bit

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
            self.clr_bits(self.REG_ADCCON0, 0x0F)
            self.set_bits(self.REG_ADCCON0, io_pin.adc_channel)
            self.i2c_write8(self.REG_AINDIDS, 0)
            self.set_bit(self.REG_AINDIDS, io_pin.adc_channel)
            self.set_bit(self.REG_ADCCON1, 0)

            self.clr_bit(self.REG_ADCCON0, 7)  # ADCF - Clear the conversion complete flag
            self.set_bit(self.REG_ADCCON0, 6)  # ADCS - Set the ADC conversion start flag

            # Wait for the ADCF conversion complete flag to be set
            t_start = time.time()
            while not self.get_bit(self.REG_ADCCON0, 7):
                time.sleep(0.01)
                if time.time() - t_start >= adc_timeout:
                    raise RuntimeError("Timeout waiting for ADC conversion!")

            hi = self.i2c_read8(self.REG_ADCRH)
            lo = self.i2c_read8(self.REG_ADCRL)
            return ((hi << 4) | lo) / 4095.0 * self._vref

        else:
            if self._debug:
                print("Reading IO from pin {}".format(pin))
            pv = self.get_bit(self.get_pin_regs(io_pin).p, io_pin.pin)

            return HIGH if pv else LOW

    def output(self, pin, value):
        """Write an IO pin state or PWM duty cycle.

        :param value: Either True/False for OUT, or a number between 0 and PWM period for PWM.

        """
        io_pin = self.get_pin(pin)

        if io_pin.mode == PIN_MODE_PWM:
            if self._debug:
                print("Outputting PWM to pin: {pin}".format(pin=pin))
            self.i2c_write8(self.get_pwm_regs(io_pin).pwml, value & 0xFF)
            self.i2c_write8(self.get_pwm_regs(io_pin).pwmh, value >> 8)
            self._pwm_load(self.get_pwm_regs(io_pin).pwmcon0)

        else:
            if value == LOW:
                if self._debug:
                    print("Outputting LOW to pin: {pin}".format(pin=pin))
                self.clr_bit(self.get_pin_regs(io_pin).p, io_pin.pin)
            elif value == HIGH:
                if self._debug:
                    print("Outputting HIGH to pin: {pin}".format(pin=pin))
                self.set_bit(self.get_pin_regs(io_pin).p, io_pin.pin)

    def get_pwm_regs(self, pin):
        return PWMRegs(
            piocon=self._regs_piocon[pin.reg_iopwm],
            pwmcon0=self._regs_pwmcon0[pin.pwm_generator],
            pwmcon1=self._regs_pwmcon1[pin.pwm_generator],
            pwml=self._regs_pwml[pin.pwm_generator][pin.pwm_channel],
            pwmh=self._regs_pwmh[pin.pwm_generator][pin.pwm_channel],
        )

    def get_pin_regs(self, pin):
        return PinRegs(
            m1=self._regs_m1[pin.port],
            m2=self._regs_m2[pin.port],
            p=self._regs_p[pin.port],
            ps=self._regs_ps[pin.port],
            int_mask_p=self._regs_int_mask_p[pin.port],
        )


class IOE(_IO, ioe_regs.REGS):
    def __init__(
        self,
        i2c_addr=None,
        interrupt_timeout=1.0,
        interrupt_pin=None,
        gpio=None,
        skip_chip_id_check=False,
    ):
        self._pins = [
            PWM_PIN(port=1, pin=5, pwm_piocon=(1, 5), pwm_channel=(0, 5)),
            PWM_PIN(port=1, pin=0, pwm_piocon=(0, 2), pwm_channel=(0, 2)),
            PWM_PIN(port=1, pin=2, pwm_piocon=(0, 0), pwm_channel=(0, 0)),
            PWM_PIN(port=1, pin=4, pwm_piocon=(1, 1), pwm_channel=(0, 1)),
            PWM_PIN(port=0, pin=0, pwm_piocon=(0, 3), pwm_channel=(0, 3)),
            PWM_PIN(port=0, pin=1, pwm_piocon=(0, 4), pwm_channel=(0, 4)),
            ADC_OR_PWM_PIN(port=1, pin=1, adc_channel=7, pwm_piocon=(0, 1), pwm_channel=(0, 1)),
            ADC_OR_PWM_PIN(port=0, pin=3, adc_channel=6, pwm_piocon=(0, 5), pwm_channel=(0, 5)),
            ADC_OR_PWM_PIN(port=0, pin=4, adc_channel=5, pwm_piocon=(1, 3), pwm_channel=(0, 3)),
            ADC_PIN(port=3, pin=0, adc_channel=1),
            ADC_PIN(port=0, pin=6, adc_channel=3),
            ADC_OR_PWM_PIN(port=0, pin=5, adc_channel=4, pwm_piocon=(1, 2), pwm_channel=(0, 2)),
            ADC_PIN(port=0, pin=7, adc_channel=2),
            ADC_PIN(port=1, pin=7, adc_channel=0),
        ]

        self._regs_m1 = [self.REG_P0M1, self.REG_P1M1, -1, self.REG_P3M1]
        self._regs_m2 = [self.REG_P0M2, self.REG_P1M2, -1, self.REG_P3M2]
        self._regs_p = [self.REG_P0, self.REG_P1, self.REG_P2, self.REG_P3]
        self._regs_ps = [self.REG_P0S, self.REG_P1S, self.REG_P2S, self.REG_P3S]
        self._regs_int_mask_p = [
            self.REG_INT_MASK_P0,
            self.REG_INT_MASK_P1,
            -1,
            self.REG_INT_MASK_P3,
        ]

        self._regs_piocon = [self.REG_PIOCON0, self.REG_PIOCON1]

        self._regs_pwmcon0 = [self.REG_PWMCON0]
        self._regs_pwmcon1 = [self.REG_PWMCON1]

        self._regs_pwmpl = [self.REG_PWMPL]
        self._regs_pwmph = [self.REG_PWMPH]

        self._regs_pwml = [
            [
                self.REG_PWM0L,
                self.REG_PWM1L,
                self.REG_PWM2L,
                self.REG_PWM3L,
                self.REG_PWM4L,
                self.REG_PWM5L,
            ],
        ]
        self._regs_pwmh = [
            [
                self.REG_PWM0H,
                self.REG_PWM1H,
                self.REG_PWM2H,
                self.REG_PWM3H,
                self.REG_PWM4H,
                self.REG_PWM5H,
            ],
        ]

        self._chip_id = self.CHIP_ID

        if i2c_addr is None:
            i2c_addr = self.I2C_ADDR

        _IO.__init__(self, i2c_addr, interrupt_timeout, interrupt_pin, gpio, skip_chip_id_check)


class SuperIOE(_IO, sioe_regs.REGS):
    def __init__(
        self,
        i2c_addr=None,
        interrupt_timeout=1.0,
        interrupt_pin=None,
        gpio=None,
        skip_chip_id_check=False,
    ):
        self._pins = [
            PIN(port=3, pin=5),
            PIN(port=3, pin=7),
            ADC_PIN(port=0, pin=6, adc_channel=3),
            ADC_PIN(port=0, pin=7, adc_channel=2),
            ADC_PIN(port=1, pin=7, adc_channel=0),
            ADC_PIN(port=3, pin=0, adc_channel=1),
            ADC_PIN(port=0, pin=4, adc_channel=5),
            ADC_PIN(port=0, pin=5, adc_channel=4),
            ADC_PIN(port=1, pin=3, adc_channel=13),
            ADC_PIN(port=2, pin=5, adc_channel=15),
            ADC_PIN(port=1, pin=1, adc_channel=7),
            ADC_PIN(port=0, pin=3, adc_channel=6),
            ADC_PIN(port=2, pin=4, adc_channel=12),
            ADC_PIN(port=2, pin=3, adc_channel=11),
            PWM_PIN(port=3, pin=3, pwm_piocon=(2, 6), pwm_channel=(0, 0)),  # NO ALT
            PWM_PIN(port=0, pin=1, pwm_piocon=(0, 4), pwm_channel=(0, 4)),  # OR PWM 3 CH 0
            PWM_PIN(port=1, pin=5, pwm_piocon=(1, 5), pwm_channel=(0, 5)),  # OR PWM 3 CH 1
            PWM_PIN(port=1, pin=4, pwm_piocon=(1, 1), pwm_channel=(0, 1)),  # OR PWM 1 CH 1
            PWM_PIN(port=0, pin=0, pwm_piocon=(0, 3), pwm_channel=(0, 3)),  # OR PWM 2 CH 1
            PWM_PIN(port=1, pin=0, pwm_piocon=(0, 2), pwm_channel=(0, 2)),  # OR PWM 2 CH 0
            PWM_PIN(port=2, pin=1, pwm_piocon=(2, 0), pwm_channel=(2, 0)),  # NO ALT
            PWM_PIN(port=2, pin=2, pwm_piocon=(2, 1), pwm_channel=(1, 1)),  # NO ALT
            PWM_PIN(port=1, pin=2, pwm_piocon=(0, 0), pwm_channel=(1, 0)),  # OR PWM 1 CH 0 (default PWM 0 CH 0)
            PWM_PIN(port=3, pin=2, pwm_piocon=(2, 5), pwm_channel=(3, 0)),  # NO ALT
            PWM_PIN(port=3, pin=4, pwm_piocon=(2, 7), pwm_channel=(3, 1)),  # NO ALT
            PWM_PIN(port=3, pin=1, pwm_piocon=(2, 4), pwm_channel=(2, 1)),  # NO ALT
        ]

        self._regs_m1 = [self.REG_P0M1, self.REG_P1M1, self.REG_P2M1, self.REG_P3M1]
        self._regs_m2 = [self.REG_P0M2, self.REG_P1M2, self.REG_P2M2, self.REG_P3M2]
        self._regs_p = [self.REG_P0, self.REG_P1, self.REG_P2, self.REG_P3]
        self._regs_ps = [self.REG_P0S, self.REG_P1S, self.REG_P2S, self.REG_P3S]
        self._regs_int_mask_p = [
            self.REG_INT_MASK_P0,
            self.REG_INT_MASK_P1,
            self.REG_INT_MASK_P2,
            self.REG_INT_MASK_P3,
        ]

        self._regs_piocon = [self.REG_PIOCON0, self.REG_PIOCON1, self.REG_PIOCON2]

        self._regs_pwmcon0 = [
            self.REG_PWM0CON0,
            self.REG_PWM1CON0,
            self.REG_PWM2CON0,
            self.REG_PWM3CON0,
        ]
        self._regs_pwmcon1 = [
            self.REG_PWM0CON1,
            self.REG_PWM1CON1,
            self.REG_PWM2CON1,
            self.REG_PWM3CON1,
        ]

        self._regs_pwmpl = [
            self.REG_PWM0PL,
            self.REG_PWM1PL,
            self.REG_PWM2PL,
            self.REG_PWM3PL,
        ]
        self._regs_pwmph = [
            self.REG_PWM0PH,
            self.REG_PWM1PH,
            self.REG_PWM2PH,
            self.REG_PWM3PH,
        ]

        self._regs_pwml = [
            [
                self.REG_PWM0C0L,
                self.REG_PWM0C1L,
                self.REG_PWM0C2L,
                self.REG_PWM0C3L,
                self.REG_PWM0C4L,
                self.REG_PWM0C5L,
            ],
            [self.REG_PWM1C0L, self.REG_PWM1C1L],
            [self.REG_PWM2C0L, self.REG_PWM2C1L],
            [self.REG_PWM3C0L, self.REG_PWM3C1L],
        ]
        self._regs_pwmh = [
            [
                self.REG_PWM0C0H,
                self.REG_PWM0C1H,
                self.REG_PWM0C2H,
                self.REG_PWM0C3H,
                self.REG_PWM0C4H,
                self.REG_PWM0C5H,
            ],
            [self.REG_PWM1C0H, self.REG_PWM1C1H],
            [self.REG_PWM2C0H, self.REG_PWM2C1H],
            [self.REG_PWM3C0H, self.REG_PWM3C1H],
        ]

        self._chip_id = self.CHIP_ID

        if i2c_addr is None:
            i2c_addr = self.I2C_ADDR

        _IO.__init__(self, i2c_addr, interrupt_timeout, interrupt_pin, gpio, skip_chip_id_check)

        # Mux p1.2 PWM over to PWM 1 Channel 0
        self.clr_bits(self.REG_AUXR4, 0b11)
        self.set_bits(self.REG_AUXR4, 0b10)
