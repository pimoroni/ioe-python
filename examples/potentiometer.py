#!/usr/bin/env python3
import time
import colorsys
import ioexpander as io


print("""potentiometer.py

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

ioe = io.IOE(i2c_addr=0x11)

ioe.set_mode(POT_ENC_A, io.PIN_MODE_PP)
ioe.set_mode(POT_ENC_B, io.PIN_MODE_PP)
ioe.set_mode(POT_ENC_C, io.ADC)

ioe.output(POT_ENC_A, 1)
ioe.output(POT_ENC_B, 0)

ioe.set_pwm_period(PERIOD)
ioe.set_pwm_control(divider=2)  # PWM as fast as we can to avoid LED flicker

ioe.set_mode(PIN_RED, io.PWM, invert=True)
ioe.set_mode(PIN_GREEN, io.PWM, invert=True)
ioe.set_mode(PIN_BLUE, io.PWM, invert=True)

print("Running LED with {} brightness steps.".format(int(PERIOD * BRIGHTNESS)))

while True:
    analog = ioe.input(POT_ENC_C)

    h = analog / 3.3  # time.time() / 10.0
    r, g, b = [int(c * PERIOD * BRIGHTNESS) for c in colorsys.hsv_to_rgb(h, 1.0, 1.0)]
    ioe.output(PIN_RED, r)
    ioe.output(PIN_GREEN, g)
    ioe.output(PIN_BLUE, b)

    print(analog, r, g, b)

    time.sleep(1.0 / 30)

