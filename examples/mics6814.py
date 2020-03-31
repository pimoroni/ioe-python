import time
from colorsys import hsv_to_rgb
import ioexpander as io

ioe = io.IOE(i2c_addr=0x18)

ioe.set_pwm_period(255)
ioe.set_mode(3, io.PWM)  # red
ioe.set_mode(7, io.PWM)  # green
ioe.set_mode(2, io.PWM)  # blue

while True:
    h = time.time() * 10.0 % 360
    r, g, b = [int(c * 255.0) for c in hsv_to_rgb(h / 360.0, 1.0, 1.0)]
    print(r, g, b)
    ioe.output(3, r)
    ioe.output(7, g)
    ioe.output(2, b)
    time.sleep(1.0 / 30)
