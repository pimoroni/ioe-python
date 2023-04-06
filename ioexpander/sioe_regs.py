class REGS:
    I2C_ADDR = 0x16
    CHIP_ID = 0x510E
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
    REG_ENC_2_CFG = 0x06
    REG_ENC_3_CFG = 0x07
    REG_ENC_4_CFG = 0x08
    REG_ENC_1_COUNT = 0x0A
    REG_ENC_2_COUNT = 0x0C
    REG_ENC_3_COUNT = 0x0E
    REG_ENC_4_COUNT = 0x10

    # Cap touch
    #REG_CAPTOUCH_EN = 0x0D
    #REG_CAPTOUCH_CFG = 0x0E
    #REG_CAPTOUCH_0 = 0x0F  # First of 8 bytes from 15-22

    # Switch counters
    REG_SWITCH_EN_P0 = 0x17
    REG_SWITCH_EN_P1 = 0x18
    REG_SWITCH_P00 = 0x19  # First of 8 bytes from 25-40
    REG_SWITCH_P10 = 0x21  # First of 8 bytes from 33-49

    REG_USER_FLASH = 0xD0
    REG_FLASH_PAGE = 0xF0

    REG_PERIPHERALS = 0xF7
    BIT_WATCHDOG = 0

    REG_DEBUG = 0xF8

    REG_P0 = 0x7B       # Bit addressing

    REG_RWKL = 0x5b
    REG_TCON = 0x55
    REG_TMOD = 0x56
    REG_TL0 = 0x4b
    REG_TL1 = 0x4d
    REG_TH0 = 0x4c
    REG_TH1 = 0x4e
    REG_CKCON = 0x4a
    REG_WKCON = 0x5a
    REG_P1 = 0x7c       # Bit addressing

    REG_CAPCON0 = 0x60
    REG_CAPCON1 = 0x61
    REG_CAPCON2 = 0x62
    REG_CKDIV = 0x47
    REG_CKSWT = 0x48    # TA protected
    REG_CKEN = 0x49     # TA protected

    REG_P2 = 0x7d       # Bit addressing
    REG_AUXR1 = 0x41

    REG_WDCON = 0x5d    # TA protected

    REG_P3M1 = 0x91
    REG_P3M2 = 0x92
    REG_P3 = 0x7e       # Bit addressing
    REG_P0M1 = 0x7f
    REG_P0M2 = 0x80
    REG_P1M1 = 0x85
    REG_P1M2 = 0x86
    REG_ADCRL = 0x6b
    REG_ADCRH = 0x6c
    REG_T3CON = 0x59
    REG_RL3 = 0x53
    REG_RH3 = 0x54
    REG_T2CON = 0x57
    REG_T2MOD = 0x58
    REG_RCMP2L = 0x51
    REG_RCMP2H = 0x52
    REG_TL2 = 0x4f
    REG_TH2 = 0x50
    REG_ADCMPL = 0x78
    REG_ADCMPH = 0x79
    REG_PWM0PH = 0x9d
    REG_PWM0C0H = 0x9f
    REG_PWM0C1H = 0xa1
    REG_PWM0C2H = 0xa3
    REG_PWM0C3H = 0xa5
    REG_PNP = 0xb1
    REG_PWM0FBD = 0xae
    REG_PWM0CON0 = 0xaa
    REG_PWM0PL = 0x9c
    REG_PWM0C0L = 0x9e
    REG_PWM0C1L = 0xa0
    REG_PWM0C2L = 0xa2
    REG_PWM0C3L = 0xa4
    REG_PIOCON0 = 0x99
    REG_PWM0CON1 = 0xab
    REG_ADCCON1 = 0x70
    REG_ADCCON2 = 0x71
    REG_ADCDLY = 0x7a
    REG_C0L = 0x65
    REG_C0H = 0x66
    REG_C1L = 0x67
    REG_C1H = 0x68
    REG_ADCCON0 = 0x6f
    REG_C2L = 0x69
    REG_C2H = 0x6a
    REG_CAPCON3 = 0x63
    REG_CAPCON4 = 0x64
    REG_SPCR = 0x42
    REG_SPSR = 0x44
    REG_SPDR = 0x45
    REG_AINDIDS0 = 0x6d
    REG_PWM0DTEN = 0xaf   # TA protected
    REG_PWM0DTCNT = 0xb0  # TA protected
    REG_PWM0MEN = 0xad
    REG_PWM0MD = 0xac
    REG_P3S = 0x95
    REG_P3SR = 0x96
    REG_P0S = 0x83
    REG_P0SR = 0x84
    REG_P1S = 0x89
    REG_P1SR = 0x8a
    REG_PWM0C4H = 0xa7
    REG_PWM0C5H = 0xa9
    REG_PIOCON1 = 0x9a
    REG_PWM0C4L = 0xa6
    REG_PWM0C5L = 0xa8
    REG_SPCR2 = 0x43
    REG_ADCBAL = 0x73
    REG_ADCBAH = 0x74
    REG_ADCCON3 = 0x72
    REG_P2M1 = 0x8b
    REG_P2M2 = 0x8c
    REG_P2SR = 0x90
    REG_P2S = 0x8f
    REG_ADCSN = 0x75
    REG_ADCCN = 0x76
    REG_ADCSR = 0x77
    REG_P0UP = 0x81
    REG_P1UP = 0x87
    REG_P2UP = 0x8d
    REG_P3UP = 0x93
    REG_RWKH = 0x5c
    REG_AINDIDS1 = 0x6e
    REG_P0DW = 0x82
    REG_P1DW = 0x88
    REG_P2DW = 0x8e
    REG_P3DW = 0x94
    REG_AUXR4 = 0x97
    REG_AUXR5 = 0x98
    REG_AUXR7 = 0x46
    REG_AUXR8 = 0x5f
    REG_PWM1PH = 0xb3
    REG_PWM1C0H = 0xb5
    REG_PWM1C1H = 0xb7
    REG_PWM1MD = 0xba
    REG_PWM1MEN = 0xbb
    REG_PWM1PL = 0xb2
    REG_PWM1C0L = 0xb4
    REG_PWM1C1L = 0xb6
    REG_PWM1CON0 = 0xb8
    REG_PWM1CON1 = 0xb9
    REG_PIOCON2 = 0x9b
    REG_PWM2PH = 0xbd
    REG_PWM2C0H = 0xbf
    REG_PWM2C1H = 0xc1
    REG_PWM2MD = 0xc4
    REG_PWM2MEN = 0xc5
    REG_PWM2PL = 0xbc
    REG_PWM2C0L = 0xbe
    REG_PWM2C1L = 0xc0
    REG_PWM2CON0 = 0xc2
    REG_PWM2CON1 = 0xc3
    REG_PWM3PH = 0xc7
    REG_PWM3C0H = 0xc9
    REG_PWM3C1H = 0xcb
    REG_PWM3MD = 0xce
    REG_PWM3MEN = 0xcf
    REG_PWM3PL = 0xc6
    REG_PWM3C0L = 0xc8
    REG_PWM3C1L = 0xca
    REG_PWM3CON0 = 0xcc
    REG_PWM3CON1 = 0xcd

    REG_INT = 0xf9
    MASK_INT_TRIG = 0x1
    MASK_INT_OUT = 0x2
    BIT_INT_TRIGD = 0
    BIT_INT_OUT_EN = 1
    BIT_INT_PIN_SWAP = 2  # 0 = P1.3, 1 = P0.0

    REG_INT_MASK_P0 = 0x00
    REG_INT_MASK_P1 = 0x01
    REG_INT_MASK_P2 = 0x02
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
