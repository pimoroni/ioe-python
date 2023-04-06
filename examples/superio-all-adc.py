#!/usr/bin/env python3
import time

import ioexpander as io

print("""superio-all-adc.py

Press Ctrl+C to exit.

""")

ADC_PINS = list(range(3, 15))

ioe = io.SuperIOE(i2c_addr=0x16)

for pin in ADC_PINS:
    ioe.set_mode(pin, io.ADC)

while True:
    print("")
    print("ADC Readings:")
    for pin in ADC_PINS:
        value = ioe.input(pin)
        print(f"    ADC {pin} {value}")

    time.sleep(1.0)
