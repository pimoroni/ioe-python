from ioexpander import IOE, OUT, LOW, HIGH

def test_cycles():
    ioe = IOE()

    # ioe.set_mode(1, PWM)
    # ioe.set_mode(9, OUT)
    ioe.set_mode(4, OUT)
    # ioe.set_mode(14, ADC)

    # ioe.output(1, 1000)

    out_value = LOW

    while True:
        a = ioe.input(4)
        # a = ioe.input(3)
        # b = ioe.input(14)

        ioe.output(4, out_value)
        out_value = not out_value

        # print("3: {a} 14: {b}".format(a=a, b=b))

        time.sleep(1.0)


if __name__ == "__main__":
    test_cycles()