import time
from colorsys import hsv_to_rgb

import ioexpander as io

ioe = io.IOE(i2c_addr=0x19)
chip_id = ioe.get_chip_id()

print("Chip ID: {:04x}".format(chip_id))

ioe.set_pwm_period(255)
ioe.set_mode(3, io.PWM)   # P1.2 LED Red
ioe.set_mode(7, io.PWM)   # P1.1 LED Green
ioe.set_mode(2, io.PWM)   # P1.0 LED Blue

ioe.set_mode(9, io.ADC)   # P0.4 AIN5 - 2v8
ioe.set_mode(12, io.ADC)  # P0.5 AIN4 - Red
ioe.set_mode(11, io.ADC)  # P0.6 AIN3 - NH3
ioe.set_mode(13, io.ADC)  # P0.7 AIN2 - OX

ioe.set_mode(1, io.PIN_MODE_OD)   # P1.5 Heater Enable
ioe.output(1, io.LOW)

last_heater = 0

while True:
    h = time.time() * 10.0 % 360
    r, g, b = [int(c * 255.0) for c in hsv_to_rgb(h / 360.0, 1.0, 1.0)]
    ioe.output(3, r)
    ioe.output(7, g)
    ioe.output(2, b)

    ref = ioe.input(9)
    red = ioe.input(12)
    nh3 = ioe.input(11)
    oxd = ioe.input(13)

    try:
        red = (red * 56000) / (ioe.get_adc_vref() - red)
    except ZeroDivisionError:
        red = 0

    try:
        nh3 = (nh3 * 56000) / (ioe.get_adc_vref() - nh3)
    except ZeroDivisionError:
        nh3 = 0

    try:
        oxd = (oxd * 56000) / (ioe.get_adc_vref() - oxd)
    except ZeroDivisionError:
        oxd = 0

    print(ref, red, nh3, oxd)

    time.sleep(1.0)
