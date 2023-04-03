from collections import namedtuple
from ioexpander import PWM
from ioexpander.common import clamp, map_float

Pair = namedtuple("Pair", ["pulse", "value"])

ANGULAR = 0
LINEAR = 1
CONTINUOUS = 2


class Calibration():
    DEFAULT_MIN_PULSE = 500.0   # in microseconds
    DEFAULT_MID_PULSE = 1500.0  # in microseconds
    DEFAULT_MAX_PULSE = 2500.0  # in microseconds

    LOWER_HARD_LIMIT = 400.0    # The minimum microsecond pulse to send
    UPPER_HARD_LIMIT = 2600.0   # The maximum microsecond pulse to send

    def __init__(self, default_type=None):
        self.calibration = None
        self.limit_lower = True
        self.limit_upper = True

        if default_type is not None:
            if not isinstance(default_type, int):
                raise TypeError("cannot convert object to an integer")

            if default_type < 0 or default_type >= 3:
                raise ValueError("type out of range. Expected ANGULAR (0), LINEAR (1) or CONTINUOUS (2)")

            self.apply_default_pairs(default_type)

    def __str__(self):
        size = self.size()
        text = f"Calibration(size = {size}, pairs = {{"
        for i in range(size):
            pair = self.calibration[i]
            text += f"{{{pair.pulse}, {pair.value}}}"
            if i < size - 1:
                text += ", "
        text += f"}}, limit_lower = {self.has_lower_limit()}, limit_upper = {self.has_upper_limit()})"
        return text

    def apply_blank_pairs(self, size):
        if size < 0:
            raise ValueError("size out of range. Expected 0 or greater")

        if self.calibration is not None:
            self.calibration = None

        if size > 0:
            self.calibration = [Pair(0, 0)] * size
        else:
            self.calibration = None

    def apply_two_pairs(self, min_pulse, max_pulse, min_value, max_value):
        self.apply_blank_pairs(2)
        self.calibration[0] = Pair(min_pulse, min_value)
        self.calibration[1] = Pair(max_pulse, max_value)

    def apply_three_pairs(self, min_pulse, mid_pulse, max_pulse, min_value, mid_value, max_value):
        self.apply_blank_pairs(3)
        self.calibration[0] = Pair(min_pulse, min_value)
        self.calibration[1] = Pair(mid_pulse, mid_value)
        self.calibration[2] = Pair(max_pulse, max_value)

    def apply_uniform_pairs(self, size, min_pulse, max_pulse, min_value, max_value):
        self.apply_blank_pairs(size)
        if size > 0:
            size_minus_one = size - 1
            for i in range(0, size):
                pulse = map_float(i, 0.0, size_minus_one, min_pulse, max_pulse)
                value = map_float(i, 0.0, size_minus_one, min_value, max_value)
                self.calibration[i] = Pair(pulse, value)

    def apply_default_pairs(self, default_type):
        if default_type == ANGULAR:
            self.apply_three_pairs(self.DEFAULT_MIN_PULSE, self.DEFAULT_MID_PULSE, self.DEFAULT_MAX_PULSE,
                                   -90.0, 0.0, +90.0)

        elif default_type == LINEAR:
            self.apply_two_pairs(self.DEFAULT_MIN_PULSE, self.DEFAULT_MAX_PULSE,
                                 0.0, 1.0)

        elif default_type == CONTINUOUS:
            self.apply_three_pairs(self.DEFAULT_MIN_PULSE, self.DEFAULT_MID_PULSE, self.DEFAULT_MAX_PULSE,
                                   -1.0, 0.0, +1.0)

    def size(self):
        return len(self.calibration)

    def pair(self, index, pair=None):  # Ensure the pairs are assigned in ascending value order
        if pair is None:
            return self.calibration[index]
        else:
            self.calibration[index] = Pair._make(pair)

    def pulse(self, index, pulse=None):
        if pulse is None:
            return self.calibration[index].pulse
        else:
            value = self.calibration[index].value
            self.calibration[index] = Pair(pulse, value)

    def value(self, index, value=None):
        if value is None:
            return self.calibration[index].value
        else:
            pulse = self.calibration[index].pulse
            self.calibration[index] = Pair(pulse, value)

    def first(self, pair=None):
        if pair is None:
            return self.calibration[0]
        else:
            self.calibration[0] = Pair._make(pair)

    def first_pulse(self, pulse=None):
        if pulse is None:
            return self.calibration[0].pulse
        else:
            value = self.calibration[0].value
            self.calibration[0] = Pair(pulse, value)

    def first_value(self, value=None):
        if value is None:
            return self.calibration[0].value
        else:
            pulse = self.calibration[0].pulse
            self.calibration[0] = Pair(pulse, value)

    def last(self, pair=None):
        if pair is None:
            return self.calibration[-1]
        else:
            self.calibration[-1] = Pair._make(pair)

    def last_pulse(self, pulse=None):
        if pulse is None:
            return self.calibration[-1].pulse
        else:
            value = self.calibration[-1].value
            self.calibration[-1] = Pair(pulse, value)

    def last_value(self, value=None):
        if value is None:
            return self.calibration[-1].value
        else:
            pulse = self.calibration[-1].pulse
            self.calibration[-1] = Pair(pulse, value)

    def has_lower_limit(self):
        return self.limit_lower

    def has_upper_limit(self):
        return self.limit_upper

    def limit_to_calibration(self, lower, upper):
        self.limit_lower = lower
        self.limit_upper = upper

    def value_to_pulse(self, value):
        if len(self.calibration) >= 2:
            last = len(self.calibration) - 1

            value_out = value

            # Is the value below the bottom most calibration pair?
            if value < self.calibration[0].value:
                # Should the value be limited to the calibration or projected below it?
                if self.limit_lower:
                    pulse_out = self.calibration[0].pulse
                    value_out = self.calibration[0].value
                else:
                    pulse_out = map_float(value, self.calibration[0].value, self.calibration[1].value,
                                                 self.calibration[0].pulse, self.calibration[1].pulse)
            # Is the value above the top most calibration pair?
            elif value > self.calibration[last].value:
                # Should the value be limited to the calibration or projected above it?
                if self.limit_upper:
                    pulse_out = self.calibration[last].pulse
                    value_out = self.calibration[last].value
                else:
                    pulse_out = map_float(value, self.calibration[last - 1].value, self.calibration[last].value,
                                                 self.calibration[last - 1].pulse, self.calibration[last].pulse)
            else:
                # The value must between two calibration pairs, so iterate through them to find which ones
                for i in range(last):
                    if value <= self.calibration[i + 1].value:
                        pulse_out = map_float(value, self.calibration[i].value, self.calibration[i + 1].value,
                                                     self.calibration[i].pulse, self.calibration[i + 1].pulse)
                        break  # No need to continue checking so break out of the loop

            # Clamp the pulse between the hard limits
            if pulse_out < self.LOWER_HARD_LIMIT or pulse_out > self.UPPER_HARD_LIMIT:
                pulse_out = clamp(pulse_out, self.LOWER_HARD_LIMIT, self.UPPER_HARD_LIMIT)

                # Is the pulse below the bottom most calibration pair?
                if pulse_out < self.calibration[0].pulse:
                    value_out = map_float(pulse_out, self.calibration[0].pulse, self.calibration[1].pulse,
                                                     self.calibration[0].value, self.calibration[1].value)

                # Is the pulse above the top most calibration pair?
                elif pulse_out > self.calibration[last].pulse:
                    value_out = map_float(pulse_out, self.calibration[last - 1].pulse, self.calibration[last].pulse,
                                                     self.calibration[last - 1].value, self.calibration[last].value)

                else:
                    # The pulse must between two calibration pairs, so iterate through them to find which ones
                    for i in range(last):
                        if pulse_out <= self.calibration[i + 1].pulse:
                            value_out = map_float(pulse_out, self.calibration[i].pulse, self.calibration[i + 1].pulse,
                                                             self.calibration[i].value, self.calibration[i + 1].value)
                            break  # No need to continue checking so break out of the loop

            return Pair(pulse_out, value_out)

        return None

    def pulse_to_value(self, pulse):
        if len(self.calibration) >= 2:
            last = len(self.calibration) - 1

            # Clamp the pulse between the hard limits
            pulse_out = clamp(pulse, self.LOWER_HARD_LIMIT, self.UPPER_HARD_LIMIT)

            # Is the pulse below the bottom most calibration pair?
            if pulse_out < self.calibration[0].pulse:
                # Should the pulse be limited to the calibration or projected below it?
                if self.limit_lower:
                    value_out = self.calibration[0].value
                    pulse_out = self.calibration[0].pulse
                else:
                    value_out = map_float(pulse, self.calibration[0].pulse, self.calibration[1].pulse,
                                                 self.calibration[0].value, self.calibration[1].value)

            # Is the pulse above the top most calibration pair?
            elif pulse > self.calibration[last].pulse:
                # Should the pulse be limited to the calibration or projected above it?
                if self.limit_upper:
                    value_out = self.calibration[last].value
                    pulse_out = self.calibration[last].pulse
                else:
                    value_out = map_float(pulse, self.calibration[last - 1].pulse, self.calibration[last].pulse,
                                                 self.calibration[last - 1].value, self.calibration[last].value)
            else:
                # The pulse must between two calibration pairs, so iterate through them to find which ones
                for i in range(last):
                    if pulse <= self.calibration[i + 1].pulse:
                        value_out = map_float(pulse, self.calibration[i].pulse, self.calibration[i + 1].pulse,
                                                     self.calibration[i].value, self.calibration[i + 1].value)
                        break  # No need to continue checking so break out of the loop

            return Pair(pulse_out, value_out)

        return None


class ServoState():
    DEFAULT_FREQUENCY = 50.0    # The standard servo update rate
    MIN_FREQUENCY = 10.0        # Lowest achievable with hardware PWM with good resolution
    MAX_FREQUENCY = 350.0       # Highest nice value that still allows the full uS pulse range
                                # Some servos are rated for 333Hz for instance
    ZERO_PERCENT = 0.0
    ONEHUNDRED_PERCENT = 1.0

    MIN_VALID_PULSE = 1.0

    def __init__(self, calibration=ANGULAR):
        self.servo_value = 0.0
        self.last_enabled_pulse = 0.0
        self.enabled = False

        if isinstance(calibration, Calibration):
            from copy import deepcopy
            self.calib = deepcopy(calibration)
        elif isinstance(calibration, int):
            self.calib = Calibration(calibration)
        else:
            raise TypeError("cannot convert object to an integer or a Calibration class instance")

    def enable_with_return(self):
        # Has the servo not had a pulse value set before being enabled?
        if self.last_enabled_pulse < self.MIN_VALID_PULSE:
            # Set the servo to its middle
            return self.to_mid_with_return()
        return self.__enable_with_return()

    def disable_with_return(self):
        self.enabled = False
        return 0.0  # A zero pulse

    def is_enabled(self):
        return self.enabled

    def __enable_with_return(self):  # Internal version of enable without convenient initialisation to the middle
        self.enabled = True
        return self.last_enabled_pulse

    def get_pulse(self):
        return self.last_enabled_pulse

    def set_pulse_with_return(self, pulse):
        if pulse >= self.MIN_VALID_PULSE:
            out = self.calib.pulse_to_value(pulse)
            if out is not None:
                self.servo_value = out.value
                self.last_enabled_pulse = out.pulse
                return self.__enable_with_return()

        return self.disable_with_return()

    def get_value(self):
        return self.servo_value

    def set_value_with_return(self, value):
        out = self.calib.value_to_pulse(value)
        if out is not None:
            self.last_enabled_pulse = out.pulse
            self.servo_value = out.value
            return self.__enable_with_return()

        return self.disable_with_return()

    def get_min_value(self):
        value = 0.0
        if self.calib.size() > 0:
            value = self.calib.first_value()
        return value

    def get_mid_value(self):
        value = 0.0
        if self.calib.size() > 0:
            first = self.calib.first_value()
            last = self.calib.last_value()
            value = (first + last) / 2.0
        return value

    def get_max_value(self):
        value = 0.0
        if self.calib.size() > 0:
            value = self.calib.last_value()
        return value

    def to_min_with_return(self):
        return self.set_value_with_return(self.get_min_value())

    def to_mid_with_return(self):
        return self.set_value_with_return(self.get_mid_value())

    def to_max_with_return(self):
        return self.set_value_with_return(self.get_max_value())

    def to_percent_with_return(self, input, in_min=ZERO_PERCENT, in_max=ONEHUNDRED_PERCENT, value_min=None, value_max=None):
        if value_min is None:
            value_min = self.get_min_value()

        if value_max is None:
            value_max = self.get_max_value()

        value = map_float(input, in_min, in_max, value_min, value_max)
        return self.set_value_with_return(value)

    def get_calibration(self):
        from copy import deepcopy
        return deepcopy(self.calib)

    def set_calibration(self, calibration):
        if not isinstance(calibration, Calibration):
            raise TypeError("cannot convert object to a Calibration class instance")

        from copy import deepcopy
        self.calib = deepcopy(calibration)

    def pulse_to_level(pulse, resolution, freq):
        level = 0
        if pulse >= ServoState.MIN_VALID_PULSE:
            level = int((pulse * resolution * freq) / 1000000)
        return level


class Servo():
    def __apply_pulse(self, pulse, load, wait_for_load):
        self.ioe.output(self.pin, ServoState.pulse_to_level(pulse, self.pwm_period, self.pwm_frequency), load, wait_for_load)

    def __init__(self, ioe, pin, calibration=ANGULAR, freq=ServoState.DEFAULT_FREQUENCY):
        self.ioe = ioe
        self.pin = pin
        self.state = ServoState(calibration)
        self.pwm_frequency = freq

        self.pin_mod = self.ioe.get_pwm_module(pin)
        self.pwm_period = self.ioe.set_pwm_frequency(self.pwm_frequency, self.pin_mod, load=False)

        ioe.set_mode(pin, PWM)
        ioe.output(pin, 0, load=True)

    def enable(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.enable_with_return(), load, wait_for_load)

    def disable(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.disable_with_return(), load, wait_for_load)

    def is_enabled(self):
        return self.state.is_enabled()

    def pulse(self, pulse=None, load=True, wait_for_load=False):
        if pulse is None:
            return self.state.get_pulse()
        else:
            self.__apply_pulse(self.state.set_pulse_with_return(pulse), load, wait_for_load)

    def value(self, value=None, load=True, wait_for_load=False):
        if value is None:
            return self.state.get_value()
        else:
            self.__apply_pulse(self.state.set_value_with_return(value), load, wait_for_load)

    def frequency(self, freq=None, load=True, wait_for_load=False):
        if freq is None:
            return self.pwm_frequency
        else:
            if (freq >= ServoState.MIN_FREQUENCY) and (freq <= ServoState.MAX_FREQUENCY):
                self.pwm_period = self.ioe.set_pwm_frequency(freq, self.pin_p_mod, load=False)
                self.pwm_frequency = freq
                if self.state.is_enabled():
                    self.__apply_pulse(self.state.get_deadzoned_duty(), self.motor_mode, load, wait_for_load)
                elif load:
                    self.ioe.pwm_load(self.pin_p_mod)
            else:
                raise ValueError(f"freq out of range. Expected {ServoState.MIN_FREQUENCY}Hz to {ServoState.MAX_FREQUENCY}Hz")

    def min_value(self):
        return self.state.get_min_value()

    def mid_value(self):
        return self.state.get_mid_value()

    def max_value(self):
        return self.state.get_max_value()

    def to_min(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.to_min_with_return(), load, wait_for_load)

    def to_mid(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.to_mid_with_return(), load, wait_for_load)

    def to_max(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.to_max_with_return(), load, wait_for_load)

    def to_percent(self, input, in_min=ServoState.ZERO_PERCENT, in_max=ServoState.ONEHUNDRED_PERCENT, value_min=None, value_max=None, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.to_percent_with_return(input, in_min, in_max, value_min, value_max), load, wait_for_load)

    def calibration(self, calibration=None):
        if calibration is None:
            return self.state.get_calibration()
        else:
            self.state.set_calibration(calibration)

    def load(self, wait_for_load=False):
        self.ioe.pwm_load(self.pin_mod, wait_for_load=wait_for_load)
