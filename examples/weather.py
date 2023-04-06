#!/usr/bin/env python3
import math
import time
from threading import Lock

import RPi.GPIO as GPIO

import ioexpander as io

# Anemometer pins
ANI1 = 5       # P0.0
ANI2 = 6       # P0.1

# Wind vane (ADC)
WV = 8         # P0.3 ANI6
WV_RADIUS = 7  # Radius from center to the center of a cup, in CM
WV_CIRCUMFERENCE = WV_RADIUS * 2 * math.pi

# Rain gauge
R2 = 3         # P1.2
R3 = 7         # P1.1
R4 = 2         # P1.0
R5 = 1         # P1.5
RAIN_MM_PER_TICK = 0.2794

# Dictionary to map compass directions to ADC values
compass_values = {
    "North": 0.9,
    "North East": 2.0,
    "East": 3.0,
    "South East": 2.8,
    "South": 2.5,
    "South West": 1.5,
    "West": 0.3,
    "North West": 0.6
}

wind = 0
rain = 0

wind_overflows = 0
rain_overflows = 0

last_wind = 0
last_rain = 0

lock = Lock()


def handle_interrupt(stuff):
    global wind, rain, last_wind, last_rain, wind_overflows, rain_overflows
    lock.acquire(blocking=True)
    ioe.clear_interrupt()
    new_wind, _ = ioe.read_switch_counter(ANI2)
    new_rain, _ = ioe.read_switch_counter(R4)

    if new_wind < last_wind:
        wind_overflows += 1
    last_wind = new_wind
    wind = (wind_overflows * 128) + new_wind

    if new_rain < last_rain:
        rain_overflows += 1
    last_rain = new_rain
    rain = (rain_overflows * 128) + new_rain

    lock.release()


ioe = io.IOE(i2c_addr=0x18, interrupt_pin=4)

# Fudge to enable pull-up on interrupt pin
ioe._gpio.setup(ioe._interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

ioe.set_adc_vref(3.3)  # Input voltage of IO Expander, this is 3.3 on Breakout Garden

# Wind Vane
ioe.set_mode(WV, io.ADC)

# Anemometer
ioe.set_mode(ANI1, io.OUT)
ioe.output(ANI1, 0)
ioe.set_pin_interrupt(ANI2, True)
ioe.setup_switch_counter(ANI2)

# Rain Sensor
ioe.set_mode(R2, io.IN_PU)
ioe.set_mode(R3, io.OUT)
ioe.set_mode(R4, io.IN_PU)
ioe.setup_switch_counter(R4)
ioe.set_mode(R5, io.IN_PU)
ioe.output(R3, 0)
ioe.set_pin_interrupt(R4, True)

ioe.on_interrupt(handle_interrupt)
ioe.clear_interrupt()

last_wind_counts = 0
last_rain_counts = 0

while True:
    lock.acquire(blocking=True)
    wind_hz = (wind - last_wind_counts) / 2.0  # Two pulses per rotation
    wind_cms = wind_hz * WV_CIRCUMFERENCE
    wind_mph = wind_cms / 44.705
    rain_hz = rain - last_rain_counts
    rain_mm_sec = rain_hz * RAIN_MM_PER_TICK
    rain_mm_total = rain * RAIN_MM_PER_TICK
    direction = ioe.input(WV)
    compass_direction, value = min(compass_values.items(), key=lambda item: abs(item[1] - direction))
    print(f"""
Wind Counts: {wind} ({int(wind_hz * 60)} RPM)
Wind Speed: {wind_cms:1.2f} cm/sec {wind_mph:1.2f} MPH
Wind Direction: {direction:1.2f} ({compass_direction})
Rain Counts: {rain} ({rain_mm_total:0.2f} mm)
Rain Speed: {rain_mm_sec} mm/sec ({rain_hz} tips/sec)
""")
    last_wind_counts = wind
    last_rain_counts = rain
    lock.release()
    time.sleep(1.0)
