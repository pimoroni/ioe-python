# Servos with IO Expander <!-- omit in toc -->

The Servo library lets you drive 3-pin hobby servo motors from Nuvoton-based boards such as the [Pimoroni IO Expander Breakout](https://shop.pimoroni.com/products/io-expander).

This library offers a `Servo` class that uses the Nuvoton's hardware PWM to drive a single servo, with support for up to 6 or 12 servos (depending on the chip size). There is also a `Calibration` class for performing advanced tweaking of each servo's movement behaviour.


## Table of Content
- [Servo](#servo)
  - [Getting Started](#getting-started)
  - [Control by Value](#control-by-value)
    - [Common Values](#common-values)
  - [Control by Percent](#control-by-percent)
  - [Control by Pulse Width](#control-by-pulse-width)
  - [Frequency Control](#frequency-control)
  - [Calibration](#calibration)
  - [Delayed Loading](#delayed-loading)
  - [Function Reference](#function-reference)
  - [PWM Limitations](#pwm-limitations)
- [Calibration](#calibration-1)
  - [Common Types](#common-types)
  - [Custom Calibration](#custom-calibration)
  - [Modifying a Calibration](#modifying-a-calibration)
  - [Movement Limits](#movement-limits)
  - [Populating a Calibration](#populating-a-calibration)
  - [Viewing the Mapping](#viewing-the-mapping)
  - [Function Reference](#function-reference-1)


## Servo

### Getting Started

To start using servos with your expander, you will need to first import the `IOE` and `Servo` classes, then create your `IOE` object.
```python
from ioexpander import IOE
from ioexpander.servo import Servo

ioe = IOE()
```
If you are using an expander board that uses the larger Nuvoton chip, then `IOE` should be replaced with `SuperIOE`.

To create your servo, choose which expander pin it will be connected to, and pass that into `Servo`.
```python
SERVO = 19
s = Servo(ioe, SERVO)
```

You now have a `Servo` class called `s` that will control the physical servo connected to pin `19`. The pin chosen must support PWM (See [PWM Limitations](#pwm-limitations)).

To start using this servo, simply enable it using:
```python
s.enable()
```

This activates the servo and moves it to it's last known position. Since this is the first time enabling the servo, there is no last known position, so instead it will move to the middle of its movement range instead.

Once you have finished with the servo, it can be disabled by calling:
```python
s.disable()
```

From here the servo can be controlled in several ways. These are covered in more detail in the following sections.


### Control by Value

The most intuitive way of controlling a servo is by value. Value can be any number that has a real-world meaning for that type of servo, for example its angle in degrees if it's a regular angular servo, or its speed as a percentage if it's a continuous rotation servo. See [Calibration](#calibration) for more details.

The value of a servo can be set by calling `.value(value)`, which takes a float as its `value` input. If the servo is disabled, this will enable it. The resulting pulse width will also be stored.

To read back the current value of the servo, call `.value()` without any input. If the servo is disabled, this will be the last value that was provided when enabled.


#### Common Values

To simplify certain motion patterns, a servo can be commanded to three common values. These are, their minimum, middle, and maximum. These are performed by calling `.to_min()`, `.to_mid()`, and `.to_max()`, respectively. If the servo is disabled, these will enable it.

It is also possible to read back the values each of those three commands is using internally, using `.min_value()`, `.mid_value()`, and `.max_value()`. These can be useful as inputs to equations that provide numbers directly to `.value(value)`, for example.


### Control by Percent

Sometimes there are projects where a servo needs to move based on the reading from a sensor or another device, but the numbers given out are not easy to convert to values the servo accepts. To overcome this the library lets you move the servo to a percent between its minimum and maximum values, or two values provided, based on that input.

With an input between `-1.0` and `1.0`, a servo can be moved to a percent between its minimum and maximum values using `.to_percent(in)`.

With an input between a provided min and max, a servo can be moved to a percent between its minimum and maximum values using `.to_percent(in, in_min, in_max)`.

With an input between a provided min and max, a servo can be moved to a percent between two provided values using `.to_percent(in, in_min, value_min, value_max)`.

If the servo is disabled, these will enable it.


### Control by Pulse Width

At a hardware level servos operate by receiving a digital signal with specific pulse widths. Typically pulses are between 500 microseconds and 2500 microseconds in length, and are usually repeated every 20 milliseconds (50Hz). These functions let you interact with pulse widths directly.

The pulse width (in microseconds) of a servo can be set by calling `.pulse(pulse)`, which takes a float as its `pulse` input. If the servo is disabled this will enable it, except for the case of `0` where instead the servo will be disabled. This function will also recalculate the related value.

To read back the current pulse width (in microseconds) of the servo, call `.pulse()` without any input. If the servo is disabled, this will be the last pulse that was provided when enabled.


### Frequency Control

The vast majority of servos expect to receive pulses with a frequency of 50Hz, so this library uses that as its default. However, there may be cases where this value needs to be changed, such as when using servos that operate up to frequencies of 333Hz.

The frequency (in Hz) of a servo can be set by calling `.frequency(freq)`, which takes a float as its `freq` input. The supported range of this input is `10` to `350` Hz.

To read back the current frequency (in Hz) of the servo, call `.frequency()` without any input.

Note that changing the frequency does not change the pulse widths sent to the servos, only how frequently they are sent. This is why `350` is the upper limit, as any higher and the maximum pulse would be longer than the time period. If you encounter any servos where this behaviour is not what they expect, please raise it as a Github issue.


### Calibration

There are different types of servos, with `ANGULAR`, `LINEAR`, and `CONTINUOUS` being common. To support these different types, each `Servo` class contains a calibration object that stores the specific value to pulse mapping needed for its type. A copy of a servo's calibration can be accessed using `.calibration()`. It is also possible to provide a servo with a new calibration using `.calibration(calibration)`.


### Delayed Loading

The `Servo` class automatically applies each change to its servo's states immediately. However, sometimes this may not be wanted, and instead you want all servos to receive updated pulses at the same time, regardless of how long the code ran that calculated the update.

For this purpose, all the functions that modify the servo state include an optional parameter `load`, which by default is `True`. To avoid this "loading" include `load=False` in the relevant function calls. Then either the last call can include `load=True`, or a specific call to `.load()` can be made.

In addition, any function that performs a load, including the `.load()` function, can be made to wait until the new PWM value has been sent out of the pins. By default this is disabled as for a regular servo this would add up to a 20ms delay, but can be enabled by including `wait_for_load=True` in the relevant function calls.


### Function Reference

Here is the complete list of functions available on the `Servo` class:
```python
Servo(pin, calibration=ANGULAR, freq=50)        # calibration can either be an integer or a Calibration class
pin()
enable(load=True, wait_for_load=False)
disable(load=True, wait_for_load=False)
is_enabled()
pulse()
pulse(pulse, load=True, wait_for_load=False)
value()
value(value, load=True, wait_for_load=False)
frequency()
frequency(freq, load=True, wait_for_load=False)
min_value()
mid_value()
max_value()
to_min(load=True, wait_for_load=False)
to_mid(load=True, wait_for_load=False)
to_max(load=True, wait_for_load=False)
to_percent(in, load=True, wait_for_load=False)
to_percent(in, in_min, in_max, load=True, wait_for_load=False)
to_percent(in, in_min, in_max, value_min, value_max, load=True, wait_for_load=False)
calibration()
calibration(calibration)
load(load=True, wait_for_load=False)
```


### PWM Limitations

The IO expander chips have limitations on the number of PWM signals, which signals can be controlled independently, and which pins can be used together:
* The regular IO expander has 6 PWM signals on a single hardware module.
* The larger IO expander has 12 PWM signals split across four hardware modules, with 6 on the first one, then 2 each on the remaining three.

A hardware module is a grouping of PWM signals that are synchronised with each other. This means that parameters such as frequency are shared, which can cause issues if you want one servo to operate at a different frequency to it's channel neighbour or to drive an LED with PWM at a high frequency. This can also be an advantage, by letting the values for multiple servos be "loaded" at the same moment.

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


## Calibration

After using servos for a while, you may notice that some don't quite go to the positions you would expect. Or perhaps you are giving values to a continuous rotation servo in degrees when it would make more sense to use a speed or rpm. To compensate for these cases, each `Servo` class has an individual `Calibration` class. This class contains a set of pulse-value pairs that are used by the `.value(value)` functions (and those similar) to convert real-world numbers into pulses each servo understand.

### Common Types

There are three common `type`s of servo's supported:
* `ANGULAR` = `0` - A servo with a value that ranges from -90 to +90 degrees.
* `LINEAR` = `1` - A servo with a value that ranges from 0 to +1.0.
* `CONTINUOUS` = `2` - A servo with a value that ranges from -1.0 to +1.0.

By default all `Servo` classes are `ANGULAR`. This can be changed by providing one of the other types as a parameter during their creation, as shown below:
```python
angular = Servo(ioe, 19)  # ANGULAR is the default so does not need to be specified here
linear = Servo(ioe, 20, LINEAR)
continuous = Servo(ioe, 21, CONTINUOUS)
```


### Custom Calibration

As well as the common types, a custom calibration can also be provided to one or more servos during creation. Below is an example that creates an angular servo that can only travel from -45 degrees to 45 degrees.

```python
cal = Calibration()
cal.apply_two_pairs(1000, 2000, -45, 45)
s = Servo(ioe, 19, cal)
```

This could be useful for example if the servo turning beyond those values would cause damage to whatever mechanism it is driving, since it would not be possible to go to angles beyond these unless limits were disabled (see [Limits](#limits)). Also it lets the exact pulse widths matching the angles be set (the `1000` and `2000` in the example). Discovering these values can take some trial and error, and will offen be different for each servo.



# Modifying a Calibration

It is also possible to access and modify the calibration of a `Servo` after their creation. This is done by first getting a copy of the servo's calibration using `.calibration()` or `.calibration(servo)`, modifying its pulses or values, then applying the modified calibration back onto to the servo.

Below, an angular servo is modified to increase its reported rotation range from 180 degrees to 270 degrees.
```python
wide_angle = Servo(ioe, 19)
cal = wide_angle.calibration()
cal.first_value(-135)
cal.last_value(+135)
wide_angle.calibration(cal)
```


### Movement Limits

As well as providing a mapping between pulses and values, the calibration class also limits a servo from going beyond its minimum and maximum values. In some cases this may not be wanted, so the state of the limits can be modified by calling `.limit_to_calibration(lower, upper)` where `lower` and `upper` are booleans. Additionally, the current state of these limits can be queried by calling `.has_lower_limit()` and `.has_upper_limit()`, respectively.

A case where you may want to disable limits is if you want a servo to go to a value (e.g. 90 degrees), but are not physically able to get a pulse measurement for that but can do another value instead (e.g. 60 degrees).

Note, even with limits disables, servos still have hard limits of `400` and `2600` microsecond pulse widths. These are intended to protect servos from receiving pulses that are too far beyond their expected range. These can vary from servo to servo though, with some hitting a physical end-stop before getting to the typical `500` and `2500` associated with -90 and +90 degrees.


### Populating a Calibration

To aid in populating a `Calibration` class, there are five helper functions that fill the class with pulse-value pairs:
* `apply_blank_pairs(size)` - Fills the calibration with the specified number of zero pairs
* `apply_two_pairs(min_pulse, max_pulse, min_value, max_value)` - Fills the calibration with two pairs using the min and max numbers provided
* `apply_three_pairs(min_pulse, mid_pulse, max_pulse, min_value, mid_value, max_value)` - Fills the calibration with three pairs using the min, mid and max numbers provided
* `apply_uniform_pairs(size, min_pulse, max_pulse, min_value, max_value)` - Fills the calibration with the specified number of pairs, interpolated from the min to max numbers provided
* `apply_default_pairs(type)` - Fills the calibration with the pairs of one of the common types

Once a `Calibration` class contains pairs (as checked `.size() > 0`), these can then be accessed by calling `.pair(index)` and can then be modified by calling `.pair(index, pair)`. The former function returns a list containing the pulse and value of the pair, and the latter accepts a list or tuple containing the pulse and value. For situations when only a single element of each pair is needed, `.pulse(index)` and `.value(index)` will return the current numbers, and `.pulse(index, pulse)` and `.value(index, value)` will override them.

For further convenience there are functions for accessing and modifying the `.first()` and `.last()` pair/pulse/value of the calibration.


### Viewing the Mapping

To aid in visualising a calibration's pulse-value mapping, the pulse for any given value can be queried by calling `.value_to_pulse(value)`. Similarly, the value for any given pulse can be queried by calling `.pulse_to_value(pulse)`. These are the same functions used by `Servo` when controlling their servos.


### Function Reference

Here is the complete list of functions available on the `Calibration` class:
```python
Calibration()
Calibration(type)
apply_blank_pairs(size)
apply_two_pairs(min_pulse, max_pulse, min_value, max_value)
apply_three_pairs(min_pulse, mid_pulse, max_pulse, min_value, mid_value, max_value)
apply_uniform_pairs(size, min_pulse, max_pulse, min_value, max_value)
apply_default_pairs(type)
size()
pair(index)
pair(index, pair)
pulse(index)
pulse(index, pulse)
value(index)
value(index, value)
first()
first(pair)
first_pulse()
first_pulse(pulse)
first_value()
first_value(value)
last()
last(pair)
last_pulse()
last_pulse(pulse)
last_value()
last_value(value)
has_lower_limit()
has_upper_limit()
limit_to_calibration(lower, upper)
value_to_pulse(value)
pulse_to_value(pulse)
```
