import colorsys
import ioexpander


class RGBLED:
    def __init__(self, ioe=None, i2c_addr=None, pin_r=1, pin_g=3, pin_b=5, invert=False, brightness=0.05):
        """RGB LED Device.

        IO Expander helper class to drive a single RGB LED connected to three PWM pins.

        Note: Will set the global PWM period and divider.

        :param ioe: Instance of ioexpander.IOE or None for automatic
        :param i2c_addr: I2C address when using automatic mode
        :param pin_r: Pin for the red LED
        :param pin_g: Pin for the green LED
        :param pin_b: Pin for the blue LED
        :param invert: Whether to invert the PWM signal (for common-cathode LEDs)
        :param brightness: LED brightness- translates to the maximum PWM duty cycle

        """
        if ioe is None:
            ioe = ioexpander.IOE(i2c_addr=i2c_addr)
        self.ioe = ioe

        # Generate a period large enough to get 255 steps
        # at the desired brightness (maximum duty cycle)
        self.period = int(255.0 / brightness)

        self.pin_r = pin_r
        self.pin_g = pin_g
        self.pin_b = pin_b

        self.brightness = brightness

        ioe.set_pwm_period(self.period)
        ioe.set_pwm_control(divider=1)

        ioe.set_mode(self.pin_r, ioexpander.PWM, invert=invert)
        ioe.set_mode(self.pin_g, ioexpander.PWM, invert=invert)
        ioe.set_mode(self.pin_b, ioexpander.PWM, invert=invert)

    def set_brightness(self, brightness):
        """Set the LED brightness.

        Re-calculates the PWM period so that colour fidelity is not lost.

        :param brightness: LED brightness (0.0 to 1.0)

        """
        self.period = int(255.0 / brightness)
        self.brightness = brightness

    def set_rgb(self, r, g, b):
        """Set the LED to an RGB colour.

        :param r: Red amount (0 to 255)
        :param g: Green amount (0 to 255)
        :param b: Blue amount (0 to 255)

        """
        r, g, b = [int(c / 255.0 * self.period * self.brightness) for c in (r, g, b)]
        self.ioe.output(self.pin_r, r)
        self.ioe.output(self.pin_g, g)
        self.ioe.output(self.pin_b, b)

    def set_hue(self, h, s=1.0, v=1.0):
        """Set the LED to a hue.

        :param h: Hue (0 to 1.0)
        :param s: Saturation (0 to 1.0)
        :param v: Value (0 to 1.0)

        """
        r, g, b = [int(c * self.period * self.brightness) for c in colorsys.hsv_to_rgb(h, s, v)]
        self.ioe.output(self.pin_r, r)
        self.ioe.output(self.pin_g, g)
        self.ioe.output(self.pin_b, b)
