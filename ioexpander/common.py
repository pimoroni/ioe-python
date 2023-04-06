NORMAL_DIR = 0
REVERSED_DIR = 1


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def map_float(input, in_min, in_max, out_min, out_max):
    return (((input - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min


# A simple class for handling Proportional, Integral & Derivative (PID) control calculations
class PID:
    def __init__(self, kp, ki, kd, sample_rate):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0
        self._error_sum = 0
        self._last_value = 0
        self._sample_rate = sample_rate

    def calculate(self, value, value_change=None):
        error = self.setpoint - value
        self._error_sum += error * self._sample_rate
        if value_change is None:
            rate_error = (value - self._last_value) / self._sample_rate
        else:
            rate_error = value_change
        self._last_value = value

        return (error * self.kp) + (self._error_sum * self.ki) - (rate_error * self.kd)
