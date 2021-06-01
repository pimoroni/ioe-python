import time

from smbus2 import SMBus, i2c_msg


__version__ = '0.0.2'


I2C_ADDR = 0x18
CHIP_ID = 0xE26A
CHIP_VERSION = 2

REG_CHIP_ID_L = 0xfa
REG_CHIP_ID_H = 0xfb
REG_VERSION = 0xfc

# Rotary encoder
REG_ENC_EN = 0x04
BIT_ENC_EN_1 = 0
BIT_ENC_MICROSTEP_1 = 1
BIT_ENC_EN_2 = 2
BIT_ENC_MICROSTEP_2 = 3
BIT_ENC_EN_3 = 4
BIT_ENC_MICROSTEP_3 = 5
BIT_ENC_EN_4 = 6
BIT_ENC_MICROSTEP_4 = 7

REG_ENC_1_CFG = 0x05
REG_ENC_1_COUNT = 0x06
REG_ENC_2_CFG = 0x07
REG_ENC_2_COUNT = 0x08
REG_ENC_3_CFG = 0x09
REG_ENC_3_COUNT = 0x0A
REG_ENC_4_CFG = 0x0B
REG_ENC_4_COUNT = 0x0C

# Cap touch
REG_CAPTOUCH_EN = 0x0D
REG_CAPTOUCH_CFG = 0x0E
REG_CAPTOUCH_0 = 0x0F  # First of 8 bytes from 15-22

# Switch counters
REG_SWITCH_EN_P0 = 0x17
REG_SWITCH_EN_P1 = 0x18
REG_SWITCH_P00 = 0x19  # First of 8 bytes from 25-40
REG_SWITCH_P10 = 0x21  # First of 8 bytes from 33-49

REG_USER_FLASH = 0xD0
REG_FLASH_PAGE = 0xF0
REG_DEBUG = 0xF8

REG_P0 = 0x40       # protect_bits 2 # Bit addressing
REG_SP = 0x41       # Read only
REG_DPL = 0x42      # Read only
REG_DPH = 0x43      # Read only
REG_RCTRIM0 = 0x44  # Read only
REG_RCTRIM1 = 0x45  # Read only
REG_RWK = 0x46
REG_PCON = 0x47     # Read only
REG_TCON = 0x48
REG_TMOD = 0x49
REG_TL0 = 0x4a
REG_TL1 = 0x4b
REG_TH0 = 0x4c
REG_TH1 = 0x4d
REG_CKCON = 0x4e
REG_WKCON = 0x4f    # Read only
REG_P1 = 0x50       # protect_bits 3 6 # Bit addressing
REG_SFRS = 0x51     # TA protected # Read only
REG_CAPCON0 = 0x52
REG_CAPCON1 = 0x53
REG_CAPCON2 = 0x54
REG_CKDIV = 0x55
REG_CKSWT = 0x56    # TA protected # Read only
REG_CKEN = 0x57     # TA protected # Read only
REG_SCON = 0x58
REG_SBUF = 0x59
REG_SBUF_1 = 0x5a
REG_EIE = 0x5b      # Read only
REG_EIE1 = 0x5c     # Read only
REG_CHPCON = 0x5f   # TA protected # Read only
REG_P2 = 0x60       # Bit addressing
REG_AUXR1 = 0x62
REG_BODCON0 = 0x63  # TA protected
REG_IAPTRG = 0x64   # TA protected # Read only
REG_IAPUEN = 0x65   # TA protected # Read only
REG_IAPAL = 0x66    # Read only
REG_IAPAH = 0x67    # Read only
REG_IE = 0x68       # Read only
REG_SADDR = 0x69
REG_WDCON = 0x6a    # TA protected
REG_BODCON1 = 0x6b  # TA protected
REG_P3M1 = 0x6c
REG_P3S = 0xc0      # Page 1 # Reassigned from 0x6c to avoid collision
REG_P3M2 = 0x6d
REG_P3SR = 0xc1     # Page 1 # Reassigned from 0x6d to avoid collision
REG_IAPFD = 0x6e    # Read only
REG_IAPCN = 0x6f    # Read only
REG_P3 = 0x70       # Bit addressing
REG_P0M1 = 0x71     # protect_bits  2
REG_P0S = 0xc2      # Page 1 # Reassigned from 0x71 to avoid collision
REG_P0M2 = 0x72     # protect_bits  2
REG_P0SR = 0xc3     # Page 1 # Reassigned from 0x72 to avoid collision
REG_P1M1 = 0x73     # protect_bits  3 6
REG_P1S = 0xc4      # Page 1 # Reassigned from 0x73 to avoid collision
REG_P1M2 = 0x74     # protect_bits  3 6
REG_P1SR = 0xc5     # Page 1 # Reassigned from 0x74 to avoid collision
REG_P2S = 0x75
REG_IPH = 0x77      # Read only
REG_PWMINTC = 0xc6  # Page 1 # Read only # Reassigned from 0x77 to avoid collision
REG_IP = 0x78       # Read only
REG_SADEN = 0x79
REG_SADEN_1 = 0x7a
REG_SADDR_1 = 0x7b
REG_I2DAT = 0x7c    # Read only
REG_I2STAT = 0x7d   # Read only
REG_I2CLK = 0x7e    # Read only
REG_I2TOC = 0x7f    # Read only
REG_I2CON = 0x80    # Read only
REG_I2ADDR = 0x81   # Read only
REG_ADCRL = 0x82
REG_ADCRH = 0x83
REG_T3CON = 0x84
REG_PWM4H = 0xc7    # Page 1 # Reassigned from 0x84 to avoid collision
REG_RL3 = 0x85
REG_PWM5H = 0xc8    # Page 1 # Reassigned from 0x85 to avoid collision
REG_RH3 = 0x86
REG_PIOCON1 = 0xc9  # Page 1 # Reassigned from 0x86 to avoid collision
REG_TA = 0x87       # Read only
REG_T2CON = 0x88
REG_T2MOD = 0x89
REG_RCMP2L = 0x8a
REG_RCMP2H = 0x8b
REG_TL2 = 0x8c
REG_PWM4L = 0xca    # Page 1 # Reassigned from 0x8c to avoid collision
REG_TH2 = 0x8d
REG_PWM5L = 0xcb    # Page 1 # Reassigned from 0x8d to avoid collision
REG_ADCMPL = 0x8e
REG_ADCMPH = 0x8f
REG_PSW = 0x90      # Read only
REG_PWMPH = 0x91
REG_PWM0H = 0x92
REG_PWM1H = 0x93
REG_PWM2H = 0x94
REG_PWM3H = 0x95
REG_PNP = 0x96
REG_FBD = 0x97
REG_PWMCON0 = 0x98
REG_PWMPL = 0x99
REG_PWM0L = 0x9a
REG_PWM1L = 0x9b
REG_PWM2L = 0x9c
REG_PWM3L = 0x9d
REG_PIOCON0 = 0x9e
REG_PWMCON1 = 0x9f
REG_ACC = 0xa0      # Read only
REG_ADCCON1 = 0xa1
REG_ADCCON2 = 0xa2
REG_ADCDLY = 0xa3
REG_C0L = 0xa4
REG_C0H = 0xa5
REG_C1L = 0xa6
REG_C1H = 0xa7
REG_ADCCON0 = 0xa8
REG_PICON = 0xa9    # Read only
REG_PINEN = 0xaa    # Read only
REG_PIPEN = 0xab    # Read only
REG_PIF = 0xac      # Read only
REG_C2L = 0xad
REG_C2H = 0xae
REG_EIP = 0xaf      # Read only
REG_B = 0xb0        # Read only
REG_CAPCON3 = 0xb1
REG_CAPCON4 = 0xb2
REG_SPCR = 0xb3
REG_SPCR2 = 0xcc    # Page 1 # Reassigned from 0xb3 to avoid collision
REG_SPSR = 0xb4
REG_SPDR = 0xb5
REG_AINDIDS = 0xb6
REG_EIPH = 0xb7     # Read only
REG_SCON_1 = 0xb8
REG_PDTEN = 0xb9    # TA protected
REG_PDTCNT = 0xba   # TA protected
REG_PMEN = 0xbb
REG_PMD = 0xbc
REG_EIP1 = 0xbe     # Read only
REG_EIPH1 = 0xbf    # Read only

REG_USER_FLASH = 0xd0
REG_FLASH_PAGE = 0xf0


REG_INT = 0xf9
MASK_INT_TRIG = 0x1
MASK_INT_OUT = 0x2
BIT_INT_TRIGD = 0
BIT_INT_OUT_EN = 1
BIT_INT_PIN_SWAP = 2  # 0 = P1.3, 1 = P0.0

REG_INT_MASK_P0 = 0x00
REG_INT_MASK_P1 = 0x01
REG_INT_MASK_P3 = 0x03


REG_VERSION = 0xfc
REG_ADDR = 0xfd

REG_CTRL = 0xfe     # 0 = Sleep, 1 = Reset, 2 = Read Flash, 3 = Write Flash, 4 = Addr Unlock
MASK_CTRL_SLEEP = 0x1
MASK_CTRL_RESET = 0x2
MASK_CTRL_FREAD = 0x4
MASK_CTRL_FWRITE = 0x8
MASK_CTRL_ADDRWR = 0x10

# Special mode registers, use a bit-addressing scheme to avoid
# writing the *whole* port and smashing the i2c pins
BIT_ADDRESSED_REGS = [REG_P0, REG_P1, REG_P2, REG_P3]

# These values encode our desired pin function: IO, ADC, PWM
# alongwide the GPIO MODE for that port and pin (section 8.1)
# the 5th bit additionally encodes the default output state
PIN_MODE_IO = 0b00000   # General IO mode, IE: not ADC or PWM
PIN_MODE_QB = 0b00000   # Output, Quasi-Bidirectional mode
PIN_MODE_PP = 0b00001   # Output, Push-Pull mode
PIN_MODE_IN = 0b00010   # Input-only (high-impedance)
PIN_MODE_PU = 0b10000   # Input (with pull-up)
PIN_MODE_OD = 0b00011   # Output, Open-Drain mode
PIN_MODE_PWM = 0b00101  # PWM, Output, Push-Pull mode
PIN_MODE_ADC = 0b01010  # ADC, Input-only (high-impedance)
MODE_NAMES = ('IO', 'PWM', 'ADC')
GPIO_NAMES = ('QB', 'PP', 'IN', 'OD')
STATE_NAMES = ('LOW', 'HIGH')

IN = PIN_MODE_IN
IN_PULL_UP = PIN_MODE_PU
IN_PU = PIN_MODE_PU
OUT = PIN_MODE_PP
PWM = PIN_MODE_PWM
ADC = PIN_MODE_ADC

HIGH = 1
LOW = 0


class PIN():
    def __init__(self, port, pin):
        if getattr(self, "type", None) is None:
            self.type = [PIN_MODE_IO]
        self.mode = None
        self.port = port
        self.pin = pin

        # The PxM1 and PxM2 registers encode GPIO MODE
        # 0 0 = Quasi-bidirectional
        # 0 1 = Push-pull
        # 1 0 = Input-only (high-impedance)
        # 1 1 = Open-drain
        self.reg_m1 = [REG_P0M1, REG_P1M1, -1, REG_P3M1][port]
        self.reg_m2 = [REG_P0M2, REG_P1M2, -1, REG_P3M2][port]

        # The Px input register
        self.reg_p = [REG_P0, REG_P1, -1, REG_P3][port]
        # The PxS Schmitt trigger register
        self.reg_ps = [REG_P0S, REG_P1S, -1, REG_P3S][port]
        self.reg_int_mask_p = [REG_INT_MASK_P0, REG_INT_MASK_P1, -1, REG_INT_MASK_P3][port]


class PWM_PIN(PIN):
    """PWM Pin.

    Class to store details of a PWM-enabled pin.

    """
    def __init__(self, port, pin, channel, reg_iopwm):
        PIN.__init__(self, port, pin)
        self.type.append(PIN_MODE_PWM)
        self.pwm_channel = channel
        self.reg_iopwm = reg_iopwm
        self.reg_pwml = [REG_PWM0L, REG_PWM1L, REG_PWM2L, REG_PWM3L, REG_PWM4L, REG_PWM5L][channel]
        self.reg_pwmh = [REG_PWM0H, REG_PWM1H, REG_PWM2H, REG_PWM3H, REG_PWM4H, REG_PWM5H][channel]


class ADC_PIN(PIN):
    """ADC Pin.

    Class to store details of an ADC-enabled pin.

    """
    def __init__(self, port, pin, channel):
        PIN.__init__(self, port, pin)
        self.type.append(PIN_MODE_ADC)
        self.adc_channel = channel


class ADC_OR_PWM_PIN(ADC_PIN, PWM_PIN):
    """ADC/PWM Pin.

    Class to store details of an ADC/PWM-enabled pin.

    """
    def __init__(self, port, pin, adc_channel, pwm_channel, reg_iopwm):
        ADC_PIN.__init__(self, port, pin, adc_channel)
        PWM_PIN.__init__(self, port, pin, pwm_channel, reg_iopwm)


class IOE():
    def __init__(self, i2c_addr=I2C_ADDR, interrupt_timeout=1.0, interrupt_pin=None, gpio=None, skip_chip_id_check=False):
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

        self._pins = [
            PWM_PIN(1, 5, 5, REG_PIOCON1),
            PWM_PIN(1, 0, 2, REG_PIOCON0),
            PWM_PIN(1, 2, 0, REG_PIOCON0),
            PWM_PIN(1, 4, 1, REG_PIOCON0),
            PWM_PIN(0, 0, 3, REG_PIOCON0),
            PWM_PIN(0, 1, 4, REG_PIOCON0),
            ADC_OR_PWM_PIN(1, 1, 7, 1, REG_PIOCON0),
            ADC_OR_PWM_PIN(0, 3, 6, 5, REG_PIOCON0),
            ADC_OR_PWM_PIN(0, 4, 5, 3, REG_PIOCON1),
            ADC_PIN(3, 0, 1),
            ADC_PIN(0, 6, 3),
            ADC_OR_PWM_PIN(0, 5, 4, 2, REG_PIOCON1),
            ADC_PIN(0, 7, 2),
            ADC_PIN(1, 7, 0)
        ]

        if not skip_chip_id_check:
            chip_id = (self.i2c_read8(REG_CHIP_ID_H) << 8) | self.i2c_read8(REG_CHIP_ID_L)
            if chip_id != CHIP_ID:
                raise RuntimeError("Chip ID invalid: {:04x} expected: {:04x}.".format(chip_id, CHIP_ID))

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

    def setup_rotary_encoder(self, channel, pin_a, pin_b, pin_c=None, count_microsteps=False):
        """Set up a rotary encoder."""
        channel -= 1
        self.set_mode(pin_a, PIN_MODE_PU, schmitt_trigger=True)
        self.set_mode(pin_b, PIN_MODE_PU, schmitt_trigger=True)
        if pin_c is not None:
            self.set_mode(pin_c, PIN_MODE_OD)
            self.output(pin_c, 0)

        self.i2c_write8([REG_ENC_1_CFG, REG_ENC_2_CFG, REG_ENC_3_CFG, REG_ENC_4_CFG][channel], pin_a | (pin_b << 4))
        self.change_bit(REG_ENC_EN, channel * 2 + 1, count_microsteps)
        self.set_bit(REG_ENC_EN, channel * 2)

    def read_rotary_encoder(self, channel):
        """Read the step count from a rotary encoder."""
        channel -= 1
        last = self._encoder_last[channel]
        reg = [REG_ENC_1_COUNT, REG_ENC_2_COUNT, REG_ENC_3_COUNT, REG_ENC_4_COUNT][channel]
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
        if reg in BIT_ADDRESSED_REGS:
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
        if reg in BIT_ADDRESSED_REGS:
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
        self.set_bit(REG_INT, BIT_INT_OUT_EN)
        self.change_bit(REG_INT, BIT_INT_PIN_SWAP, pin_swap)

    def disable_interrupt_out(self):
        """Disable the IOE interrupt output."""
        self.clr_bit(REG_INT, BIT_INT_OUT_EN)

    def get_interrupt(self):
        """Get the IOE interrupt state."""
        if self._interrupt_pin is not None:
            return self._gpio.input(self._interrupt_pin) == 0
        else:
            return self.get_bit(REG_INT, BIT_INT_TRIGD)

    def clear_interrupt(self):
        """Clear the interrupt flag."""
        self.clr_bit(REG_INT, BIT_INT_TRIGD)

    def set_pin_interrupt(self, pin, enabled):
        """Enable/disable the input interrupt on a specific pin.

        :param pin: Pin from 1-14
        :param enabled: True/False for enabled/disabled

        """
        if pin < 1 or pin > len(self._pins):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = self._pins[pin - 1]

        self.change_bit(io_pin.reg_int_mask_p, io_pin.pin, enabled)

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
        self.set_bit(REG_CTRL, 4)
        self.i2c_write8(REG_ADDR, i2c_addr)
        self._i2c_addr = i2c_addr
        time.sleep(0.25)  # TODO Handle addr change IOError better
        # self._wait_for_flash()
        self.clr_bit(REG_CTRL, 4)

    def set_adc_vref(self, vref):
        """Set the ADC voltage reference."""
        self._vref = vref

    def get_adc_vref(self):
        """Get the ADC voltage reference."""
        return self._vref

    def get_chip_id(self):
        """Get the IOE chip ID."""
        return (self.i2c_read8(REG_CHIP_ID_H) << 8) | self.i2c_read8(REG_CHIP_ID_L)

    def _pwm_load(self):
        # Load new period and duty registers into buffer
        t_start = time.time()
        self.set_bit(REG_PWMCON0, 6)    # Set the "LOAD" bit of PWMCON0
        while self.get_bit(REG_PWMCON0, 6):
            time.sleep(0.001)           # Wait for "LOAD" to complete
            if time.time() - t_start >= self._timeout:
                raise RuntimeError("Timed out waiting for PWM load!")

    def set_pwm_control(self, divider):
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
                128: 0b111}[divider]
        except KeyError:
            raise ValueError("A clock divider of {}".format(divider))

        # TODO: This currently sets GP, PWMTYP and FBINEN to 0
        # It might be desirable to make these available to the user
        # GP - Group mode enable (changes first three pairs of pAM to PWM01H and PWM01L)
        # PWMTYP - PWM type select: 0 edge-aligned, 1 center-aligned
        # FBINEN - Fault-break input enable

        self.i2c_write8(REG_PWMCON1, pwmdiv2)

    def set_pwm_period(self, value):
        """Set the PWM period.

        The period is the point at which the PWM counter is reset to zero.

        The PWM clock runs at FSYS with a divider of 1/1.

        Also specifies the maximum value that can be set in the PWM duty cycle.

        """
        value &= 0xffff
        self.i2c_write8(REG_PWMPL, value & 0xff)
        self.i2c_write8(REG_PWMPH, value >> 8)

        self._pwm_load()

    def get_mode(self, pin):
        """Get the current mode of a pin."""
        return self._pins[pin - 1].mode

    def set_mode(self, pin, mode, schmitt_trigger=False, invert=False):
        """Set a pin output mode.

        :param mode: one of the supplied IN, OUT, PWM or ADC constants

        """
        if pin < 1 or pin > len(self._pins):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = self._pins[pin - 1]
        if io_pin.mode == mode:
            return

        gpio_mode = mode & 0b11
        io_mode = (mode >> 2) & 0b11
        initial_state = mode >> 4

        if io_mode != PIN_MODE_IO and mode not in io_pin.type:
            raise ValueError("Pin {} does not support {}!".format(pin, MODE_NAMES[io_mode]))

        io_pin.mode = mode
        if self._debug:
            print("Setting pin {pin} to mode {mode} {name}, state: {state}".format(pin=pin, mode=MODE_NAMES[io_mode], name=GPIO_NAMES[gpio_mode], state=STATE_NAMES[initial_state]))

        if mode == PIN_MODE_PWM:
            self.set_bit(io_pin.reg_iopwm, io_pin.pwm_channel)
            self.change_bit(REG_PNP, io_pin.pwm_channel, invert)
            self.set_bit(REG_PWMCON0, 7)  # Set PWMRUN bit

        else:
            if PIN_MODE_PWM in io_pin.type:
                self.clr_bit(io_pin.reg_iopwm, io_pin.pwm_channel)

        pm1 = self.i2c_read8(io_pin.reg_m1)
        pm2 = self.i2c_read8(io_pin.reg_m2)

        # Clear the pm1 and pm2 bits
        pm1 &= 255 - (1 << io_pin.pin)
        pm2 &= 255 - (1 << io_pin.pin)

        # Set the new pm1 and pm2 bits according to our gpio_mode
        pm1 |= (gpio_mode >> 1) << io_pin.pin
        pm2 |= (gpio_mode & 0b1) << io_pin.pin

        self.i2c_write8(io_pin.reg_m1, pm1)
        self.i2c_write8(io_pin.reg_m2, pm2)

        # Set up Schmitt trigger mode on inputs
        if mode in [PIN_MODE_PU, PIN_MODE_IN]:
            self.change_bit(io_pin.reg_ps, io_pin.pin, schmitt_trigger)

        # 5th bit of mode encodes default output pin state
        self.i2c_write8(io_pin.reg_p, (initial_state << 3) | io_pin.pin)

    def input(self, pin, adc_timeout=1):
        """Read the IO pin state.

        Returns a 12-bit ADC reading if the pin is in ADC mode
        Returns True/False if the pin is in any other input mode
        Returns None if the pin is in PWM mode

        :param adc_timeout: Timeout (in seconds) for an ADC read (default 1.0)

        """
        if pin < 1 or pin > len(self._pins):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = self._pins[pin - 1]

        if io_pin.mode == PIN_MODE_ADC:
            if self._debug:
                print("Reading ADC from pin {}".format(pin))
            self.clr_bits(REG_ADCCON0, 0x0f)
            self.set_bits(REG_ADCCON0, io_pin.adc_channel)
            self.i2c_write8(REG_AINDIDS, 0)
            self.set_bit(REG_AINDIDS, io_pin.adc_channel)
            self.set_bit(REG_ADCCON1, 0)

            self.clr_bit(REG_ADCCON0, 7)  # ADCF - Clear the conversion complete flag
            self.set_bit(REG_ADCCON0, 6)  # ADCS - Set the ADC conversion start flag

            # Wait for the ADCF conversion complete flag to be set
            t_start = time.time()
            while not self.get_bit(REG_ADCCON0, 7):
                time.sleep(0.01)
                if time.time() - t_start >= adc_timeout:
                    raise RuntimeError("Timeout waiting for ADC conversion!")

            hi = self.i2c_read8(REG_ADCRH)
            lo = self.i2c_read8(REG_ADCRL)
            return ((hi << 4) | lo) / 4095.0 * self._vref

        else:
            if self._debug:
                print("Reading IO from pin {}".format(pin))
            pv = self.get_bit(io_pin.reg_p, io_pin.pin)

            return HIGH if pv else LOW

    def output(self, pin, value):
        """Write an IO pin state or PWM duty cycle.

        :param value: Either True/False for OUT, or a number between 0 and PWM period for PWM.

        """
        if pin < 1 or pin > len(self._pins):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = self._pins[pin - 1]

        if io_pin.mode == PIN_MODE_PWM:
            if self._debug:
                print("Outputting PWM to pin: {pin}".format(pin=pin))
            self.i2c_write8(io_pin.reg_pwml, value & 0xff)
            self.i2c_write8(io_pin.reg_pwmh, value >> 8)
            self._pwm_load()

        else:
            if value == LOW:
                if self._debug:
                    print("Outputting LOW to pin: {pin}".format(pin=pin))
                self.clr_bit(io_pin.reg_p, io_pin.pin)
            elif value == HIGH:
                if self._debug:
                    print("Outputting HIGH to pin: {pin}".format(pin=pin))
                self.set_bit(io_pin.reg_p, io_pin.pin)
