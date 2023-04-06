#!/usr/bin/env python3
import colorsys
import time

import ioexpander as io

print("""pir.py

Passive infra-red.

Press Ctrl+C to exit.

""")

PIN_PIR_SENS = 1
PIN_PIR_OUT = 9
PIN_PIR_OEN = 6
PIN_RED = 3
PIN_GREEN = 7
PIN_BLUE = 2

BRIGHTNESS = 0.05               # Effectively the maximum fraction of the period that the LED will be on
PERIOD = int(255 / BRIGHTNESS)  # Add a period large enough to get 0-255 steps at the desired brightness

ioe = io.IOE(i2c_addr=0x18)

ioe.set_pwm_period(PERIOD)
ioe.set_pwm_control(divider=1)  # PWM as fast as we can to avoid LED flicker


ioe.set_mode(PIN_PIR_SENS, io.OUT)
ioe.set_mode(PIN_PIR_OUT, io.ADC)
ioe.set_mode(PIN_PIR_OEN, io.OUT)

ioe.output(PIN_PIR_SENS, 0)
ioe.output(PIN_PIR_OEN, 1)

ioe.set_mode(PIN_RED, io.PWM, invert=True)
ioe.set_mode(PIN_GREEN, io.PWM, invert=True)
ioe.set_mode(PIN_BLUE, io.PWM, invert=True)

print("Running LED with {} brightness steps.".format(int(PERIOD * BRIGHTNESS)))

while True:
    value = ioe.input(PIN_PIR_OUT)
    h = time.time() / 10.0
    r, g, b = [int(c * PERIOD * BRIGHTNESS) for c in colorsys.hsv_to_rgb(h, 1.0, 1.0)]
    print(value, r, g, b)
    if value < 0.5:
        r, g, b = 0, 0, 0
    ioe.output(PIN_RED, r)
    ioe.output(PIN_GREEN, g)
    ioe.output(PIN_BLUE, b)

    time.sleep(1.0 / 30)
