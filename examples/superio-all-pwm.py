#!/usr/bin/env python3
import time

import ioexpander as io

print("""superio-all-pwm.py

Press Ctrl+C to exit.

""")

PWM_PINS = list(range(15, 27))

PERIOD = 256

ioe = io.SuperIOE(i2c_addr=0x16)

for pwm in range(4):
    ioe.set_pwm_period(PERIOD, pwm_module=pwm)
    ioe.set_pwm_control(divider=16, pwm_module=pwm)

for pin in PWM_PINS:
    ioe.set_mode(pin, io.PWM)


for index, pin in enumerate(PWM_PINS):
    duty = PERIOD // (len(PWM_PINS) + 1) * (index + 1)
    duty_percent = duty * 100 / PERIOD
    ioe.output(pin, duty)
    print(f"Setting pin {pin} to {duty} ({duty_percent}%)")

while True:
    time.sleep(1.0 / 30)
