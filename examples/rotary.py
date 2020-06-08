#!/usr/bin/env python3
import time
import colorsys
import ioexpander as io


print("""rotary.py

Press Ctrl+C to exit.

""")

PIN_RED = 1
PIN_GREEN = 7
PIN_BLUE = 2

POT_ENC_A = 12
POT_ENC_B = 3
POT_ENC_C = 11

BRIGHTNESS = 0.5                # Effectively the maximum fraction of the period that the LED will be on
PERIOD = int(255 / BRIGHTNESS)  # Add a period large enough to get 0-255 steps at the desired brightness

ioe = io.IOE(i2c_addr=0x18, interrupt_pin=4)

ioe.enable_interrupt_out(pin_swap=True)

ioe.set_mode(POT_ENC_A, io.PIN_MODE_PU)
ioe.set_mode(POT_ENC_B, io.PIN_MODE_PU)
ioe.set_mode(POT_ENC_C, io.PIN_MODE_OD)
ioe.output(POT_ENC_C, 0)

ioe.set_bit(io.REG_P0S, 5)  # Set P0.5 = A as schmitt trigger input
ioe.set_bit(io.REG_P1S, 2)  # Set P1.2 = B as schmitt trigger input

ioe.i2c_write8(io.REG_ENC_1_CFG, POT_ENC_A | (POT_ENC_B << 4))
ioe.set_bit(io.REG_ENC_EN, io.BIT_ENC_MICROSTEP_1)
ioe.set_bit(io.REG_ENC_EN, io.BIT_ENC_EN_1)

ioe.set_pwm_period(PERIOD)
ioe.set_pwm_control(divider=2)  # PWM as fast as we can to avoid LED flicker

ioe.set_mode(PIN_RED, io.PWM, invert=True)
ioe.set_mode(PIN_GREEN, io.PWM, invert=True)
ioe.set_mode(PIN_BLUE, io.PWM, invert=True)

print("Running LED with {} brightness steps.".format(int(PERIOD * BRIGHTNESS)))

count = 0

while True:
    if ioe.get_interrupt():
        count = ioe.i2c_read8(io.REG_ENC_1_COUNT)
        if count & 0b10000000:
            count -= 255
        ioe.clear_interrupt()
    h = time.time() / 10.0
    r, g, b = [int(c * PERIOD * BRIGHTNESS) for c in colorsys.hsv_to_rgb(h, 1.0, 1.0)]
    ioe.output(PIN_RED, r)
    ioe.output(PIN_GREEN, g)
    ioe.output(PIN_BLUE, b)

    print(count, r, g, b)

    time.sleep(1.0 / 30)

