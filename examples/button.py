#!/usr/bin/env python3
import time

import ioexpander as io

print("""button.py

Demonstrates handling a single button input using IO Expander.

Your button should be wired between pin 14 and ground on the IO Expander.

A pull-up resistor will be enabled, causing your button to read 1 normally, and 0 when pressed.

Press Ctrl+C to exit.

""")

ioe = io.IOE(i2c_addr=0x18)
ioe.set_mode(14, io.IN_PU)

last_value = io.HIGH

while True:
    value = ioe.input(14)

    if value != last_value:
        print("Button has been {}".format("released" if value else "pressed"))
        last_value = value

    time.sleep(1.0 / 30)
