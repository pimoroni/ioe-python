import time

import ioexpander as io


def test_cycles():
    ioe = io.IOE()

    # ioe.set_mode(1, io.PWM)
    # ioe.set_mode(9, io.OUT)
    ioe.set_mode(4, io.OUT)
    # ioe.set_mode(14, io.ADC)

    # ioe.output(1, 1000)

    out_value = io.LOW

    while True:
        a = ioe.input(3)
        b = ioe.input(14)

        ioe.output(4, out_value)
        out_value = not out_value

        print("3: {a} 14: {b}".format(a=a, b=b))

        time.sleep(1.0)


if __name__ == "__main__":
    test_cycles()
