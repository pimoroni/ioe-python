#!/usr/bin/env python3
import colorsys
import time

import ioexpander as io

print("""superio-pwm.py

Demonstrates running a common-cathode RGB LED, or trio of LEDs wired between each PWM pin and Ground.

You must wire your Red, Green and Blue LEDs or LED elements to pins 15, 19 and 23.

Press Ctrl+C to exit.

""")

PIN_RED = 15
PIN_GREEN = 19
PIN_BLUE = 23

BRIGHTNESS = 0.05               # Effectively the maximum fraction of the period that the LED will be on
PERIOD = int(255 / BRIGHTNESS)  # Add a period large enough to get 0-255 steps at the desired brightness

ioe = io.SuperIOE(i2c_addr=0x16)

ioe.set_pwm_period(PERIOD)
ioe.set_pwm_control(divider=1)  # PWM as fast as we can to avoid LED flicker

# Since pin 23 uses PWM1, we have to set the period/control for PWM1 too
ioe.set_pwm_period(PERIOD, pwm_generator=1)
ioe.set_pwm_control(divider=1, pwm_generator=1)

ioe.set_mode(PIN_RED, io.PWM)
ioe.set_mode(PIN_GREEN, io.PWM)
ioe.set_mode(PIN_BLUE, io.PWM)

print("Running LED with {} brightness steps.".format(int(PERIOD * BRIGHTNESS)))

while True:
    h = time.time() / 10.0
    r, g, b = [int(c * PERIOD * BRIGHTNESS) for c in colorsys.hsv_to_rgb(h, 1.0, 1.0)]
    print(r, g, b)
    ioe.output(PIN_RED, r)
    ioe.output(PIN_GREEN, g)
    ioe.output(PIN_BLUE, b)

    time.sleep(1.0 / 30)
