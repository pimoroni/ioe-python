__version__ = '0.0.1'


I2C_ADDR = 0x0F
CHIP_ID = 0xE26A
CHIP_VERSION = 1

REG_P0 = 0x40 
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
REG_P1 = 0x50 
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
REG_P2 = 0x60 
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
REG_P3 = 0x70 
REG_P0M1 = 0x71 
REG_P0S = 0xc2      # Page 1 # Reassigned from 0x71 to avoid collision 
REG_P0M2 = 0x72 
REG_P0SR = 0xc3     # Page 1 # Reassigned from 0x72 to avoid collision 
REG_P1M1 = 0x73 
REG_P1S = 0xc4      # Page 1 # Reassigned from 0x73 to avoid collision 
REG_P1M2 = 0x74 
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

# These values encode our desired pin function: IO, ADC, PWM
# alongwide the GPIO MODE for that port and pin (section 8.1)
PIN_MODE_IO = 0b0000   # General IO mode, IE: not ADC or PWM
PIN_MODE_QB = 0b0000   # Output, Quasi-Bidirectional mode
PIN_MODE_PP = 0b0011   # Output, Push-Pull mode
PIN_MODE_IN = 0b0010   # Input-only (high-impedance)
PIN_MODE_OD = 0b0011   # Output, Open-Drain mode
PIN_MODE_PWM = 0b0101  # PWM, Output, Push-Pull mode
PIN_MODE_ADC = 0b1010  # ADC, Input-only (high-impedance)
MODE_NAMES = ('IO', 'PWM', 'ADC')

IN = PIN_MODE_IN
OUT = PIN_MODE_PP
PWM = PIN_MODE_PWM
ADC = PIN_MODE_ADC

HIGH = 1
LOW = 0


class PIN():
    def __init__(self, port, pin):
        self.type = PIN_MODE_IO
        self.mode = PIN_MODE_IO
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


class PWM_PIN(PIN):
    def __init__(self, port, pin, channel, reg_iopwm):
        PIN.__init__(self, port, pin)
        self.type = PIN_MODE_PWM
        self.channel = channel
        self.reg_iopwm = reg_iopwm
        self.reg_pwml = [REG_PWM0L, REG_PWM1L, REG_PWM2L, REG_PWM3L, REG_PWM4L, REG_PWM5L][channel]
        self.reg_pwmh = [REG_PWM0H, REG_PWM1H, REG_PWM2H, REG_PWM3H, REG_PWM4H, REG_PWM4H][channel]


class ADC_PIN(PIN):
    def __init__(self, port, pin, channel):
        PIN.__init__(self, port, pin)
        self.type = PIN_MODE_ADC
        self.channel = channel


PINS = [
    PWM_PIN(1, 5, 5, REG_PIOCON1),
    PWM_PIN(1, 0, 2, REG_PIOCON0),
    PWM_PIN(1, 2, 0, REG_PIOCON0),
    PWM_PIN(1, 4, 1, REG_PIOCON1),
    PWM_PIN(0, 0, 3, REG_PIOCON0),
    PWM_PIN(0, 1, 4, REG_PIOCON0),
    ADC_PIN(1, 1, 7),
    ADC_PIN(0, 3, 6),
    ADC_PIN(0, 4, 5),
    ADC_PIN(3, 0, 1),
    ADC_PIN(0, 6, 3),
    ADC_PIN(0, 5, 4),
    ADC_PIN(0, 7, 2),
    ADC_PIN(1, 7, 0)
]


class IOE():
    def __init__(self, i2c_addr=I2C_ADDR, i2c_dev=None):
        self._i2c_addr = i2c_addr
        self._i2c_dev = i2c_dev
        if self._i2c_dev is None:
            import smbus
            self._i2c_dev = smbus.SMBus(1)

    def i2c_read8(self, reg):
        return self._i2c_dev.read_i2c_block_data(self._i2c_addr, reg, 1)[0]

    def i2c_write8(self, reg, value):
        self._i2c_dev.write_i2c_block_data(self._i2c_addr, reg, [value])

    def set_bits(self, reg, bits):
        value = self.i2c_read8(reg)
        self.i2c_write8(reg, value | bits)

    def set_bit(self, reg, bit):
        self.set_bits(reg, (1 << bit))

    def clr_bits(self, reg, bits):
        value = self.i2c_read8(reg)
        self.i2c_write8(reg, value & ~bits)

    def clr_bit(self, reg, bit):
        self.clr_bits(reg, (1 << bit))

    def get_bit(self, reg, bit):
        return self.i2c_read8(reg) & (1 << bit)

    def set_mode(self, pin, mode):
        io_pin = PINS[pin - 1]
        if io_pin.mode == mode:
            return

        gpio_mode = mode & 0b11
        io_mode =  mode >> 2

        if io_mode != PIN_MODE_IO and mode != io_pin.type:
            raise ValueError("Pin {} does not support {}!".format(pin, MODE_NAMES[mode]))

        io_pin.mode = mode

        if mode == PIN_MODE_PWM:
            self.set_bit(io_pin.reg_iopwm, io_pin.channel)

        else:
            if io_pin.type == PIN_MODE_PWM:
                self.clr_bit(io_pin.reg_iopwm, io_pin.channel)


        pm1 = self.i2c_read8(io_pin.reg_m1)
        pm2 = self.i2c_read8(io_pin.reg_m2)

        # Clear the pm1 and pm2 bits
        pm1 &= ~(1 << io_pin.pin)
        pm2 &= ~(1 << io_pin.pin)

        # Set the new pm1 and pm2 bits according to our gpio_mode
        pm1 |= (gpio_mode >> 1) << io_pin.pin
        pm2 |= (gpio_mode & 0b1) << io_pin.pin

        self.i2c_write8(io_pin.reg_m1, pm1)
        self.i2c_write8(io_pin.reg_m2, pm2)

    def input(self, pin):
        """Read the IO pin state.
        
        Returns a 12-bit ADC reading if the pin is in ADC mode
        Returns True/False if the pin is in any other input mode
        Returns None if the pin is in PWM mode
        
        """
        if pin < 1 or pin > len(PINS):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = PINS[pin - 1]

        if io_pin.mode == PIN_MODE_ADC:
            self.clr_bits(REG_ADCCON0, 0x0f)
            self.set_bits(REG_ADCCON0, io_pin.channel)
            self.i2c_write8(REG_AINDIDS, 0)
            self.set_bit(REG_AINDIDS, io_pin.channel)
            self.set_bit(REG_ADCCON1, 0)

            self.clr_bit(REG_ADCCON0, 7)  # ADCF - Clear the conversion complete flag
            self.set_bit(REG_ADCCON0, 6)  # ADCS - Set the ADC conversion start flag

            # Wait for the ADCF conversion complete flag to be set
            while not self.get_bit(REG_ADCCON0, 7):
                time.sleep(0.001)

            hi = self.i2c_read8(REG_ADCRH)
            lo = self.i2c_read8(REG_ADCRL)
            return ((hi << 4) | lo) / 4095.0 * 5.0

        elif io_pin.mode != PIN_MODE_PWM:
            pv = self.i2c_read8(io_pin.reg_p)

            return HIGH if pv & (1 << io_pin.pin) else LOW

        # Fall-through for PWM mode
        return None
    
    def output(self, pin, value):
        if pin < 1 or pin > len(PINS):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = PINS[pin - 1]

        if io_pin.type == PIN_MODE_PWM:
            self.i2c_write8(io_pin.reg_pwml, value & 0xff)
            self.i2c_write8(io_pin.reg_pwmh, value >> 8)
            self.set_bit(REG_PWMCON0, 6)  # Set the "LOAD" bit of PWMCON0
                                          # Loads new period and duty registers into buffer
            while self.get_bit(REG_PWMCON0, 6):
                pass                      # Wait for "LOAD" to complete

        elif io_pin.type != PIN_MODE_IP:
            if value == LOW:
                self.clr_bit(io_pin.reg_p, io_pin.pin)
            elif value == HIGH:
                self.set_bit(io_pin.reg_p, io_pin.pin)


def test_cycles():
    import time

    ioe = IOE()

    ioe.set_mode(1, PWM)
    ioe.set_mode(2, OUT)
    ioe.set_mode(3, IN)
    ioe.set_mode(14, ADC)

    ioe.output(1, 1000)

    out_value = True

    while True:
        a = ioe.input(3)
        b = ioe.input(14)
        ioe.output(2, out_value)
        out_value = not out_value

        print("3: {a} 14: {b}".format(a=a, b=b))

        time.sleep(1.0)


if __name__ == "__main__":
    test_cycles()
