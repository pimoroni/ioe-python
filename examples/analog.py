#!/usr/bin/env python3
import time

import ioexpander as io

print("""analog.py

Demonstrates handling a single analog input using IO Expander.

You should wire a rotary potentiometer to pin 12.

The pot should be connected to 3.3v power and ground.

Press Ctrl+C to exit.

""")

ioe = io.IOE(i2c_addr=0x18)

ioe.set_adc_vref(3.3)  # Input voltage of IO Expander, this is 3.3 on Breakout Garden
ioe.set_mode(12, io.ADC)

last_adc = 0.00

while True:
    adc = ioe.input(12)
    adc = round(adc, 2)

    if adc != last_adc:
        print("{:.2f}v".format(adc))
        last_adc = adc

    time.sleep(1.0 / 30)
