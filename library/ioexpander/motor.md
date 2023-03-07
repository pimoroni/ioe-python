# Motors with IO Expander<!-- omit in toc -->

The Motor library lets you drive DC motors from Nuvoton-based boards such as the [Pimoroni IO Expander Breakout](https://shop.pimoroni.com/products/io-expander) via connected h-bridge drivers.

This library offers a `Motor` class that uses the Nuvoton's hardware PWM to drive a single motor, with support for up to 3 or 6 motors (depending on the chip size).


## Table of Content
- [Motor](#motor)
  - [Getting Started](#getting-started)
  - [Control by Speed](#control-by-speed)
    - [Full Speed](#full-speed)
    - [Stopping](#stopping)
    - [Calibration](#calibration)
  - [Control by Percent](#control-by-percent)
  - [Control by Duty Cycle](#control-by-duty-cycle)
    - [Duty Deadzone](#duty-deadzone)
  - [Frequency Control](#frequency-control)
  - [Configuration](#configuration)
    - [Direction](#direction)
    - [Decay Mode](#decay-mode)
    - [Driver Type](#driver-type)
  - [Delayed Loading](#delayed-loading)
  - [Function Reference](#function-reference)
  - [Constants Reference](#constants-reference)
  - [PWM Limitations](#pwm-limitations)


## Motor

### Getting Started

To start using motors with your expander, you will need to first import the `IOE` and `Motor` classes, then create your `IOE` object.
```python
from ioexpander import IOE
from ioexpander.motor import Motor

ioe = IOE()
```
If you are using an expander board that uses the larger Nuvoton chip, then `IOE` should be replaced with `SuperIOE`.

To create your motor, choose which expander pins it will be connected to, and pass that into `Motor`.
```python
MOTOR_POS = 19
MOTOR_NEG = 20
m = Motor(ioe, (MOTOR_POS, MOTOR_NEG))
```

You now have a `Motor` class called `m` that will control the physical motor connected to pin `19` and `20`. The pins chosen must support PWM and be on the same PWM hardware module (See [PWM Limitations](#pwm-limitations)).

To start using this motor, simply enable it using:
```python
m.enable()
```

This activates the motor and sets it to its last known speed. Since this is the first time enabling the motor, there is no last known speed, so instead it will be zero.

Once you have finished with the motor, it can be disabled by calling:
```python
m.disable()
```

From here the motor can be controlled in several ways. These are covered in more detail in the following sections.


### Control by Speed

The most intuitive way of controlling a motor is by speed. Speed can be any number that has a real-world meaning for that type of motor, for example revolutions per minute, or the linear or angular speed of the mechanism it is driving. By default the speed is a value ranging from `-1.0` to `1.0` but this can be changed by setting a new `speed_scale`. See [Calibration](#calibration) for more details.

The speed of a motor can be set by calling `.speed(speed)`, which takes a float as its `speed` input. If the motor is disabled, this will enable it. The resulting duty cycle will also be stored.

To read back the current speed of the motor, call `.speed()` without any input. If the motor is disabled, this will be the last speed that was provided when enabled.


#### Full Speed

To simplify certain motion patterns, a motor can be commanded to its full negative, and full positive speeds. These are performed by calling `.full_negative()`, and `.full_positive()`, respectively. If the motor is disabled, these will enable it.

The value of the full negative and full positive speed can be read back using `.speed_scale()`. This can be useful as an input to equations that provide numbers directly to `.speed(speed)`, for example.


#### Stopping

The easiest way to stop a motor is by calling `.stop()`. This is equivalent to calling `.speed(0.0)` and stops the motor using the currently assigned decay mode of the `Motor` object. See [Decay Mode](#decay-mode) for more details.

It is also possible to explicitly have the motor coast or brake to a stop by calling `.coast()` or `.brake()`.

If the motor is disabled, these will enable it.


#### Calibration

It is very rare for a motor to perfectly drive at the speed we want them to. As such, the `Motor` class offers two parameters for adjusting how the value provided to `.speed(speed)` is converted to the PWM duty cycle that is actually sent to the motor, a speed scale, and a zeropoint.

Speed scale, as the name implies, is a value that scales the duty cycle up or down to better reflect the measured speed of the motor when driving at full speed. This can be set by calling `.speed_scale(speed_scale)`, which accepts a value greater than `0.0`. The current speed scale can also be read by calling `.speed_scale()`.

Zeropoint is a value that sets what duty cycle should count as the zero speed of the motor. By default this is `0.0` and usually it is fine to leave it at that, but there are cases at low speeds where the expected speed does not match the measured speed, which small adjustments to the zeropoint will fix. This can be set by calling `.zeropoint(zeropoint)`, which accepts a value from `0.0` to less than `1.0`. The current zeropoint can also be read by calling `.zeropoint()`.

Both parameters can also be provided during the creation of a new `Motor` object.


### Control by Percent

Sometimes there are projects where a motor needs to drive based on the reading from a sensor or another device, but the numbers given out are not easy to convert to speeds the motor accepts. To overcome this the library lets you drive the motor at a percent between its negative and positive speeds, or two speeds provided, based on that input.

With an input between `-1.0` and `1.0`, a motor can be set to a percent between its negative and positive speeds using `.to_percent(in)`.

With an input between a provided min and max, a motor can be set to a percent between its negative and positive speeds using `.to_percent(in, in_min, in_max)`.

With an input between a provided min and max, a motor can be set to a percent between two provided speeds using `.to_percent(in, in_min, value_min, value_max)`.

If the motor is disabled, these will enable it.


### Control by Duty Cycle

Motor drivers accept pulse width modulated (PWM) signals to control the speed and direction of their connected motors. The percentage of time that these signals are active for is know as their duty cycle. This is typically measured as a value between `0.0` and `1.0`, but as motors use two pins for their control signals, here negative values are added to denote the reverse direction.

The duty cycle of a motor can be set by calling `.duty(duty)`, which takes a float from `-1.0` to `1.0` as its `duty` input. If the motor is disabled this will enable it. This function will also recalculate the related speed.

To read back the current duty cycle of the motor, call `.duty()` without any input. If the motor is disabled, this will be the last duty that was provided when enabled.


#### Duty Deadzone

Most motors have a duty cycle value below which their is too much friction for them to move. This may not be a concern, except for when running motors at audable frequencies, where the buzzing of the motor trying to move without success can get annoying.

To overcome this, a duty cycle deadzone can be set on a per motor basis by calling `.deadzone(deadzone)`, which accepts a float from `0.0` to less than `1.0`. Whenever a duty cycle is set, either directly or via speed or percent functions, it will only be output to the motor if it is greater than or equal to the deadzone. If it's below, the motor will be stopped instead. By default the deadzone is `0.05`.

To read back the current duty deadzone of the motor, call `.deadzone()` without any input.

Note that the motor keeps track of its duty cycle, so if the deadzone is changed the check will be performed again, and may either start a motor that was previously stopped or visa versa.


### Frequency Control

Motors can be driven at a variety of frequencies, with common values being above the range of human hearing. As such this library uses 25KHz as its default, but this can easily be changed.

The frequency (in Hz) of a motor can be set by calling `.frequency(freq)`, which takes a float as its `freq` input. The supported range of this input is `10` Hz to `400` KHz, though not all motor drivers can handle the very high frequencies.

To read back the current frequency (in Hz) of the motor, call `.frequency()` without any input.

Note that changing the frequency does not change the duty cycle or speed sent to the motors, only how frequently pulses are sent.

### Configuration

#### Direction

The driving direction of a motor can be changed either by providing `direction=REVERSED_DIR` when creating the `Motor` object, or by calling `.direction(REVERSED_DIR)` at any time in code. The `REVERSED_DIR` constant comes from the `ioexpander.common` module. There is also a `NORMAL_DIR` constant, though this is the default.

The current direction of a motor can be read back by calling `.direction()`.


#### Decay Mode

If you have ever had a motor directly connected to a power source and turned the power off, or disconnected the wire, you may have noticed that the motor continues to spin for a second or two before it reaches a stop. This is because the magnetic field the power source was generating has decayed away quickly, so the only thing slowing the motor down is friction. This results in the motor coasting to a stop.

By contrast, if you were to wire your circuit up such that instead of disconnecting the power, the off position joined the two ends of the motor together, it would take longer for the magnetic field to decay away. This has the effect of braking the motor, causing it to stop quicker than with friction alone.

These examples describe the two decay modes supported by the `Motor` class, `FAST_DECAY`, and `SLOW_DECAY`, respectively. Generally slow decay offers better motor performance, particularly with low speeds, so this is the default when creating a new `Motor`.

If fast decay is wanted then it can either be changed by providing `mode=FAST_DECAY` during the class creation or by calling `.decay_mode(FAST_DECAY)`. The current decay mode can also be read with `.decay_mode()`.

For more information about motor decay modes, it's highly recommended that you check out the Adafruit Learn Guide titled [Improve Brushed DC Motor Performance](https://learn.adafruit.com/improve-brushed-dc-motor-performance)


### Delayed Loading

The `Motor` class automatically applies each change to its motor's states immediately. However, sometimes this may not be wanted, and instead you want all motors to receive updated duty cycles at the same time, regardless of how long the code ran that calculated the update.

For this purpose, all the functions that modify the motor state include an optional parameter `load`, which by default is `True`. To avoid this "loading" include `load=False` in the relevant function calls. Then either the last call can include `load=True`, or a specific call to `.load()` can be made.

In addition, any function that performs a load, including the `.load()` function, can be made to wait until the new PWM value has been sent out of the pins. By default this is disabled, but can be enabled by including `wait_for_load=True` in the relevant function calls.


### Function Reference

Here is the complete list of functions available on the `Motor` class:
```python
Motor(ioe, pins, direction=NORMAL_DIR, speed_scale=1.0, zeropoint=0.0, deadzone=0.05, freq=25000, mode=SLOW_DECAY)
enable(load=True, wait_for_load=False)
disable(load=True, wait_for_load=False)
is_enabled()
duty()
duty(duty, load=True, wait_for_load=False)
speed()
speed(speed, load=True, wait_for_load=False)
frequency()
frequency(freq, load=True, wait_for_load=False)
stop(load=True, wait_for_load=False)
coast(load=True, wait_for_load=False)
brake(load=True, wait_for_load=False)
full_negative(load=True, wait_for_load=False)
full_positive(load=True, wait_for_load=False)
to_percent(in, load=True, wait_for_load=False)
to_percent(in, in_min, in_max, load=True, wait_for_load=False)
to_percent(in, in_min, in_max, speed_min, speed_max, load=True, wait_for_load=False)
direction()
direction(direction)
speed_scale()
speed_scale(speed_scale)
zeropoint()
zeropoint(zeropoint)
deadzone()
deadzone(deadzone, load=True, wait_for_load=False)
decay_mode()
decay_mode(mode, load=True, wait_for_load=False)
load(wait_for_load=False)
```

### Constants Reference

Here is the complete list of constants on the `ioexpander.motor` module:

* `FAST_DECAY` = `0`
* `SLOW_DECAY` = `1`

Here are useful constants from the `ioexpander.common` module:

* `NORMAL_DIR` = `0`
* `REVERSED_DIR` = `1`


### PWM Limitations

The IO expander chips have limitations on the number of PWM signals, which signals can be controlled independently, and which pins can be used together:
* The regular IO expander has 6 PWM signals on a single hardware module.
* The larger IO expander has 12 PWM signals split across four hardware modules, with 6 on the first one, then 2 each on the remaining three.

A hardware module is a grouping of PWM signals that are synchronised with each other. This means that parameters such as frequency are shared, which can cause issues if you want one motor to operate at a different frequency to it's channel neighbour or to drive an LED with PWM at a high frequency. This can also be an advantage, by letting the values for multiple motors be "loaded" at the same moment.

The PWM pinout for the regular IO expander is shown below. Note how some of the PWM channels are repeated:

| Pin             | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 |
|-----------------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|
| PWM Channel     | 5 | 2 | 0 | 1 | 3 | 4 | 1 | 5 | 3 | -  | -  | 2  | -  | -  |


The PWM pinout for the larger IO expander is shown below. Note how some of the PWM channels are repeated, and how some pins have alternate PWMs they can be assigned to:

| Pin             | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 |
|-----------------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|
| PWM Module      | - | - | - | - | 3 | 0 | 0 | 0 | - | -  | 0  | 0  | -  | 1  |
| PWM Channel     | - | - | - | - | 2 | 1 | 3 | 2 | - | -  | 1  | 5  | -  | 0  |
|-----------------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|
| Alt PWM Module  | - | - | - | - | - | - | 2 | 2 | - | -  | 1  | 3  | -  | -  |
| Alt PWM Channel | - | - | - | - | - | - | 1 | 0 | - | -  | 1  | 1  | -  | -  |

| Pin             | 15 | 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 | 24 | 25 | 26 |
|-----------------|----|----|----|----|----|----|----|----|----|----|----|----|
| PWM Module      | 0  | 0  | 0  | 0  | 0  | 0  | 2  | 1  | 0  | 3  | 3  | 2  |
| PWM Channel     | 0  | 4  | 5  | 1  | 3  | 2  | 0  | 1  | 0  | 0  | 1  | 1  |
|-----------------|----|----|----|----|----|----|----|----|----|----|----|----|
| Alt PWM Module  | -  | 3  | 3  | 1  | 2  | 2  | -  | -  | 1  | -  | -  | -  |
| Alt PWM Channel | -  | 0  | 1  | 1  | 1  | 0  | -  | -  | 0  | -  | -  | -  |
