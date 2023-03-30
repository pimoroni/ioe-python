class REGS:
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
    REG_AINDIDS0 = 0xb6
    REG_AINDIDS1 = None # Added to have common code with SuperIO
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
