#!/usr/bin/env python3
import math
import time

import ioexpander as io

print("""servo.py

Demonstrates running a servo at 50Hz

Press Ctrl+C to exit.

""")

PIN_PWM = 1

# Settings to produce a 50Hz output from the 24MHz clock.
# 24,000,000 Hz / 8 = 3,000,000 Hz
# 3,000,000 Hz / 60,000 Period = 50 Hz
DIVIDER = 8
PERIOD = 60000

ioe = io.IOE(i2c_addr=0x18)

ioe.set_pwm_period(PERIOD)
ioe.set_pwm_control(divider=DIVIDER)

ioe.set_mode(PIN_PWM, io.PWM)


while True:
    t = time.time() / 10.0
    s = (math.sin(t * math.pi) + 1) / 2.0
    servo_us = 1000 + (s * 1000)                 # Between 1000 and 2000us (1-2ms)

    duty_per_microsecond = PERIOD / (20 * 1000)  # Default is 3 LSB per microsecond

    duty_cycle = int(round(servo_us * duty_per_microsecond))
    print("Pulse {:.2f}us, Duty Cycle: {}".format(servo_us, duty_cycle))

    ioe.output(PIN_PWM, duty_cycle)

    time.sleep(1.0 / 60)
