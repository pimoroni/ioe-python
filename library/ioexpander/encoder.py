import time
from sys import float_info
from collections import namedtuple
from ioexpander.common import NORMAL_DIR, REVERSED_DIR
from math import pi

MMME_CPR = 12
ROTARY_CPR = 24


Capture = namedtuple("Capture", ["count",
                                 "delta",
                                 "frequency",
                                 "revolutions",
                                 "degrees",
                                 "radians",
                                 "revolutions_delta",
                                 "degrees_delta",
                                 "radians_delta",
                                 "revolutions_per_second",
                                 "revolutions_per_minute",
                                 "degrees_per_second",
                                 "radians_per_second"])


class Encoder():
    def __init__(self, ioe, channel, pins, common_pin=None, direction=NORMAL_DIR, counts_per_rev=ROTARY_CPR, count_microsteps=False, count_divider=1):
        self.ioe = ioe
        self.channel = channel
        self.enc_direction = direction
        self.enc_counts_per_rev = counts_per_rev
        self.enc_count_divider = count_divider

        if not isinstance(pins, list) and not isinstance(pins, tuple):
            raise TypeError("cannot convert object to a list or tuple of pins")

        if len(pins) != 2:
            raise TypeError("list or tuple must only contain two integers")

        self.local_count = 0
        self.local_step = 0
        self.local_turn = 0
        self.last_raw_count = 0
        self.last_delta_count = 0
        self.last_capture_count = 0
        self.last_capture_time = time.monotonic_ns()

        self.ioe.setup_rotary_encoder(channel, pins[0], pins[1], pin_c=common_pin, count_microsteps=count_microsteps)

    def __take_reading(self):
        # Read the current count
        raw_count = self.ioe.read_rotary_encoder(self.channel) // self.enc_count_divider
        raw_change = raw_count - self.last_raw_count
        self.last_raw_count = raw_count

        # Invert the change
        if self.enc_direction == REVERSED_DIR:
            raw_change = 0 - raw_change

        self.local_count += raw_change

        self.local_step += raw_change
        if raw_change > 0:
            while self.local_step >= self.enc_counts_per_rev:
                self.local_step -= self.enc_counts_per_rev
                self.local_turn += 1

        elif raw_change < 0:
            while self.local_step < 0:
                self.local_step += self.enc_counts_per_rev
                self.local_turn -= 1

    def count(self):
        self.__take_reading()
        return self.local_count

    def delta(self):
        self.__take_reading()

        # Determine the change in counts since the last time this function was performed
        change = self.local_count - self.last_delta_count
        self.last_delta_count = self.local_count

        return change

    def zero(self):
        self.ioe.clear_rotary_encoder(self.channel)
        self.local_count = 0
        self.local_step = 0
        self.local_turn = 0
        self.last_raw_count = 0
        self.last_delta_count = 0
        self.last_capture_count = 0
        self.last_capture_time = time.monotonic_ns()

    def step(self):
        self.__take_reading()
        return self.local_step

    def turn(self):
        self.__take_reading()
        return self.local_turn

    def revolutions(self):
        return self.count() / self.enc_counts_per_rev

    def degrees(self):
        return self.revolutions() * 360.0

    def radians(self):
        return self.revolutions() * pi * 2.0

    def direction(self, direction=None):
        if direction is None:
            return self.enc_direction
        else:
            if direction not in (NORMAL_DIR, REVERSED_DIR):
                raise ValueError("direction out of range. Expected NORMAL_DIR (0) or REVERSED_DIR (1)")
            self.enc_direction = direction

    def counts_per_rev(self, counts_per_rev=None):
        if counts_per_rev is None:
            return self.enc_counts_per_rev
        else:
            if counts_per_rev < float_info.epsilon:
                raise ValueError("counts_per_rev out of range. Expected greater than 0.0")
            self.enc_counts_per_rev = counts_per_rev

    def capture(self):
        capture_time = time.monotonic_ns()
        self.__take_reading()

        # Determine the change in counts since the last capture was taken
        change = self.local_count - self.last_capture_count
        self.last_capture_count = self.local_count

        frequency = 0.0
        if capture_time > self.last_capture_time:  # Sanity check (shouldn't really happen)
            frequency = (float(change) * 1000000000) / float(capture_time - self.last_capture_time)
        self.last_capture_time = capture_time

        revolutions = self.local_count / self.enc_counts_per_rev
        revolutions_delta = change / self.enc_counts_per_rev
        revolutions_per_second = frequency / self.enc_counts_per_rev

        return Capture(count=self.local_count,
                       delta=change,
                       frequency=frequency,
                       revolutions=revolutions,
                       degrees=revolutions * 360.0,
                       radians=revolutions * pi * 2.0,
                       revolutions_delta=revolutions_delta,
                       degrees_delta=revolutions_delta * 360.0,
                       radians_delta=revolutions_delta * pi * 2.0,
                       revolutions_per_second=revolutions_per_second,
                       revolutions_per_minute=revolutions_per_second * 60.0,
                       degrees_per_second=revolutions_per_second * 360.0,
                       radians_per_second=revolutions_per_second * pi * 2.0)
