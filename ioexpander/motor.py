from sys import float_info
from ioexpander import PWM
from ioexpander.common import clamp, map_float, NORMAL_DIR, REVERSED_DIR

FAST_DECAY = 0  # aka 'Coasting'
SLOW_DECAY = 1  # aka 'Braking'


class MotorState():
    DEFAULT_SPEED_SCALE = 1.0  # The standard motor speed scale
    DEFAULT_ZEROPOINT = 0.0  # The standard motor zeropoint
    DEFAULT_DEADZONE = 0.05  # The standard motor deadzone

    DEFAULT_DECAY_MODE = SLOW_DECAY  # The standard motor decay behaviour
    DEFAULT_FREQUENCY = 25000.0  # The standard motor update rate
    MIN_FREQUENCY = 3.0
    MAX_FREQUENCY = 12000000.0

    ZERO_PERCENT = 0.0
    ONEHUNDRED_PERCENT = 1.0

    def __init__(self, direction=NORMAL_DIR, speed_scale=DEFAULT_SPEED_SCALE, zeropoint=DEFAULT_ZEROPOINT, deadzone=DEFAULT_DEADZONE):
        self.motor_speed = 0.0
        self.last_enabled_duty = 0.0
        self.enabled = False

        self.motor_direction = direction
        self.motor_scale = max(speed_scale, float_info.epsilon)
        self.motor_zeropoint = clamp(zeropoint, 0.0, 1.0 - float_info.epsilon)
        self.motor_deadzone = clamp(deadzone, 0.0, 1.0)

    def enable_with_return(self):
        self.enabled = True
        return self.get_deadzoned_duty()

    def disable_with_return(self):
        self.enabled = False
        return None

    def is_enabled(self):
        return self.enabled

    def get_duty(self):
        if self.motor_direction == NORMAL_DIR:
            return self.last_enabled_duty
        else:
            return 0.0 - self.last_enabled_duty

    def get_deadzoned_duty(self):
        duty = 0.0
        if self.last_enabled_duty <= 0.0 - self.motor_deadzone or self.last_enabled_duty >= self.motor_deadzone:
            duty = self.last_enabled_duty

        if self.enabled:
            return duty
        else:
            return None

    def set_duty_with_return(self, duty):
        # Invert provided speed if the motor direction is reversed
        if self.motor_direction == REVERSED_DIR:
            duty = 0.0 - duty

        # Clamp the duty between the hard limits
        self.last_enabled_duty = clamp(duty, -1.0, 1.0)

        # Calculate the corresponding speed
        self.motor_speed = MotorState.__duty_to_speed(self.last_enabled_duty, self.motor_zeropoint, self.motor_scale)

        return self.enable_with_return()

    def get_speed(self):
        if self.motor_direction == NORMAL_DIR:
            return self.motor_speed
        else:
            return 0.0 - self.motor_speed

    def set_speed_with_return(self, speed):
        # Invert provided speed if the motor direction is reversed
        if self.motor_direction == REVERSED_DIR:
            speed = 0.0 - speed

        # Clamp the speed between the hard limits
        self.motor_speed = clamp(speed, 0.0 - self.motor_scale, self.motor_scale)

        # Calculate the corresponding duty cycle
        self.last_enabled_duty = MotorState.__speed_to_duty(self.motor_speed, self.motor_zeropoint, self.motor_scale)

        return self.enable_with_return()

    def stop_with_return(self):
        return self.set_duty_with_return(0.0)

    def full_negative_with_return(self):
        return self.set_duty_with_return(-1.0)

    def full_positive_with_return(self):
        return self.set_duty_with_return(1.0)

    def to_percent_with_return(self, input, in_min=ZERO_PERCENT, in_max=ONEHUNDRED_PERCENT, speed_min=None, speed_max=None):
        if speed_min is None:
            speed_min = 0.0 - self.motor_scale
        if speed_max is None:
            speed_max = self.motor_scale

        speed = map_float(input, in_min, in_max, speed_min, speed_max)
        return self.set_speed_with_return(speed)

    def get_direction(self):
        return self.motor_direction

    def set_direction(self, direction):
        self.motor_direction = direction

    def get_speed_scale(self):
        return self.motor_scale

    def set_speed_scale(self, speed_scale):
        self.motor_scale = max(speed_scale, float_info.epsilon)
        self.motor_speed = MotorState.__duty_to_speed(self.last_enabled_duty, self.motor_zeropoint, self.motor_scale)

    def get_zeropoint(self):
        return self.motor_zeropoint

    def set_zeropoint(self, zeropoint):
        self.motor_zeropoint = clamp(zeropoint, 0.0, 1.0 - float_info.epsilon)
        self.motor_speed = MotorState.__duty_to_speed(self.last_enabled_duty, self.motor_zeropoint, self.motor_scale)

    def get_deadzone(self):
        return self.motor_deadzone

    def set_deadzone_with_return(self, deadzone):
        self.motor_deadzone = clamp(deadzone, 0.0, 1.0)
        return self.get_deadzoned_duty()

    def duty_to_level(duty, resolution):
        return int(duty * resolution)

    def __duty_to_speed(duty, zeropoint, scale):
        speed = 0.0
        if duty > zeropoint:
            speed = map_float(duty, zeropoint, 1.0, 0.0, scale)
        elif duty < -zeropoint:
            speed = map_float(duty, -zeropoint, -1.0, 0.0, -scale)
        return speed

    def __speed_to_duty(speed, zeropoint, scale):
        duty = 0.0
        if speed > 0.0:
            duty = map_float(speed, 0.0, scale, zeropoint, 1.0)
        elif speed < 0.0:
            duty = map_float(speed, 0.0, -scale, -zeropoint, -1.0)
        return duty


class Motor():

    def __apply_duty(self, duty, mode, load, wait_for_load):
        if duty is not None:
            signed_level = MotorState.duty_to_level(duty, self.pwm_period)

            if mode == SLOW_DECAY:  # aka 'Braking'
                if signed_level >= 0:
                    self.ioe.output(self.pin_p, self.pwm_period, load=False)
                    self.ioe.output(self.pin_n, self.pwm_period - signed_level, load=load, wait_for_load=wait_for_load)
                else:
                    self.ioe.output(self.pin_p, self.pwm_period + signed_level, load=False)
                    self.ioe.output(self.pin_n, self.pwm_period, load=load, wait_for_load=wait_for_load)

            elif mode == FAST_DECAY:  # aka 'Coasting'
                if signed_level >= 0:
                    self.ioe.output(self.pin_p, signed_level, load=False)
                    self.ioe.output(self.pin_n, 0, load=load, wait_for_load=wait_for_load)
                else:
                    self.ioe.output(self.pin_p, 0, load=False)
                    self.ioe.output(self.pin_n, 0 - signed_level, load=load, wait_for_load=wait_for_load)

        else:
            self.ioe.output(self.pin_p, 0, load=False)
            self.ioe.output(self.pin_n, 0, load=load, wait_for_load=wait_for_load)

    def __init__(self, ioe, pins, direction=NORMAL_DIR, speed_scale=MotorState.DEFAULT_SPEED_SCALE, zeropoint=MotorState.DEFAULT_ZEROPOINT,
                 deadzone=MotorState.DEFAULT_DEADZONE, freq=MotorState.DEFAULT_FREQUENCY, mode=MotorState.DEFAULT_DECAY_MODE):

        self.ioe = ioe

        if not isinstance(pins, list) and not isinstance(pins, tuple):
            raise TypeError("cannot convert object to a list or tuple of pins")

        if len(pins) != 2:
            raise TypeError("list or tuple must only contain two integers")

        self.pin_p = pins[0]
        self.pin_n = pins[1]

        self.state = MotorState(direction, speed_scale, zeropoint, deadzone)

        self.pwm_frequency = freq
        self.motor_mode = mode

        self.pin_p_mod = self.ioe.get_pwm_module(self.pin_p)
        self.pin_n_mod = self.ioe.get_pwm_module(self.pin_n)
        if self.pin_p_mod != self.pin_n_mod:
            raise ValueError("Both motor pins must be on the same PWM module!")

        self.pwm_period = self.ioe.set_pwm_frequency(self.pwm_frequency, self.pin_p_mod, load=False)

        ioe.set_mode(self.pin_p, PWM)
        ioe.set_mode(self.pin_n, PWM)
        ioe.output(self.pin_p, 0, load=False)
        ioe.output(self.pin_n, 0, load=True)

    def enable(self, load=True, wait_for_load=False):
        self.__apply_duty(self.state.enable_with_return(), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def disable(self, load=True, wait_for_load=False):
        self.__apply_duty(self.state.disable_with_return(), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def is_enabled(self):
        return self.state.is_enabled()

    def duty(self, duty=None, load=True, wait_for_load=False):
        if duty is None:
            return self.state.get_duty()
        else:
            self.__apply_duty(self.state.set_duty_with_return(duty), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def speed(self, speed=None, load=True, wait_for_load=False):
        if speed is None:
            return self.state.get_speed()
        else:
            self.__apply_duty(self.state.set_speed_with_return(speed), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def frequency(self, freq=None, load=True, wait_for_load=False):
        if freq is None:
            return self.pwm_frequency
        else:
            if (freq >= MotorState.MIN_FREQUENCY) and (freq <= MotorState.MAX_FREQUENCY):
                self.pwm_period = self.ioe.set_pwm_frequency(freq, self.pin_p_mod, load=False)
                self.pwm_frequency = freq
                self.__apply_duty(self.state.get_deadzoned_duty(), self.motor_mode, load=load, wait_for_load=wait_for_load)
            else:
                raise ValueError(f"freq out of range. Expected {MotorState.MIN_FREQUENCY}Hz to {MotorState.MAX_FREQUENCY}Hz")

    def stop(self, load=True, wait_for_load=False):
        self.__apply_duty(self.state.stop_with_return(), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def coast(self, load=True, wait_for_load=False):
        self.__apply_duty(self.state.stop_with_return(), FAST_DECAY, load=load, wait_for_load=wait_for_load)

    def brake(self, load=True, wait_for_load=False):
        self.__apply_duty(self.state.stop_with_return(), SLOW_DECAY, load=load, wait_for_load=wait_for_load)

    def full_negative(self, load=True, wait_for_load=False):
        self.__apply_duty(self.state.full_negative_with_return(), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def full_positive(self, load=True, wait_for_load=False):
        self.__apply_duty(self.state.full_positive_with_return(), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def to_percent(self, input, in_min=MotorState.ZERO_PERCENT, in_max=MotorState.ONEHUNDRED_PERCENT, speed_min=None, speed_max=None, load=True, wait_for_load=False):
        self.__apply_duty(self.state.to_percent_with_return(input, in_min, in_max, speed_min, speed_max), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def direction(self, direction=None):
        if direction is None:
            return self.state.get_direction()
        else:
            self.state.set_direction(direction)

    def speed_scale(self, speed_scale=None):
        if speed_scale is None:
            return self.state.get_speed_scale()
        else:
            self.state.set_speed_scale(speed_scale)

    def zeropoint(self, zeropoint=None):
        if zeropoint is None:
            return self.state.get_zeropoint()
        else:
            self.state.set_zeropoint(zeropoint)

    def deadzone(self, deadzone=None, load=True, wait_for_load=False):
        if deadzone is None:
            return self.state.get_deadzone()
        else:
            self.__apply_duty(self.state.set_deadzone_with_return(deadzone), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def decay_mode(self, mode=None, load=True, wait_for_load=False):
        if mode is None:
            return self.motor_mode
        else:
            self.motor_mode = mode
            self.__apply_duty(self.state.get_deadzoned_duty(), self.motor_mode, load=load, wait_for_load=wait_for_load)

    def load(self, wait_for_load=False):
        self.ioe.pwm_load(self.pin_p_mod, wait_for_load=wait_for_load)
