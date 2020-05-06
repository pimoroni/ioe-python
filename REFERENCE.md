# Pimoroni IO Expander

The Pimoroni IO Expander is based upon a Nuvoton MS51 and exposes much of the chip's functionality for reading/writing general IO, ADC and PWM.

- [Pimoroni IO Expander](#pimoroni-io-expander)
  - [Pins](#pins)
  - [Functions](#functions)
    - [General-purpose IO](#general-purpose-io)
    - [Analog Inputs (ADC)](#analog-inputs-adc)
    - [Pulse Width Modulation Outputs (PWM)](#pulse-width-modulation-outputs-pwm)

## Pins

All pins are capable as functioning as general purpose inputs and outputs, but may additionally function as a PWM output and/or ADC input.

1. P1.5 - PWM
2. P1.0 - PWM
3. P1.2 - PWM
4. P1.4 - PWM
5. P0.0 - PWM
6. P0.1 - PWM
7. P1.1 - ADC or PWM
8. P0.3 - ADC or PWM
9. P0.4 - ADC or PWM
10. P3.0 - ADC
11. P0.6 - ADC
12. P0.5 - ADC or PWM
13. P0.7 - ADC
14. P1.7 - ADC

## Functions

In all cases you will need to create an instance of the `IOE` class to manage your IO expander:

```python
import ioexpander

ioe = ioexpander.IOE()
```

### General-purpose IO

All pins support general-purpose IO.

For input pins you will usually use `ioexpander.IN` which sets pins to Quasi-Bidirectional mode.

You can also use `ioexpander.PIN_MODE_IN` for input-only high-impedance mode.

To set a pin as an input:

```python
ioe.set_mode(1, ioexpander.IN)
```

And read its value:

```python
value = ioe.input(1)
```

For output pins you may choose a variety of output modes:

* `ioexpander.PIN_MODE_PP` - Output, Push-Pull mode. Drives a pin either HIGH or LOW.
* `ioexpander.PIN_MODE_OD` - Output, Open-Drain mode. Drives low, or leaves the pin floating

To set a pin as an output:

```python
ioe.set_mode(1, ioexpander.OUT)
```

And set its value:

```python
ioe.output(1, 1)  # High (Low in OD)
ioe.output(1, 1)  # Low (Floating in OD)
```

Note: when using open-drain mode (`PIN_MODE_OD`), writing a `1` will pull the pin low and writing a `0` will leave the pin floating.

### Analog Inputs (ADC)

Pins 7, 8, 9 10, 11, 12, 13, and 14 support analog input.

IO Expander's `input` method will automatically give you a voltage for pins configured in ADC mode:

```python
ioe.set_mode(7, ioexpander.ADC)
voltage = ioe.input(7)
```

This is scaled against the ADC vref value, which can be read/set with:

```python
ioe.set_adc_vref(5)
vref = ioe.get_adc_vref()
```

For accurate analog readings, the vref value (which defaults to 3.3) should match the voltage at which the breakout is being powered. In most cases this will be either 3.3v or 5v.

### Pulse Width Modulation Outputs (PWM)

Pins 1, 2, 4, 5, and 6 support PWM output as marked. Additionally pins 7, 8, 9 and 12 (marked as ADC on the IO expander) can be configured as PWM outputs.

PWM, by default, uses the 24MHz FSYS clock and has  16bit period and duty-cycle registers.

There are 8 dividers available to slow the clock input into the PWM generator:

* 1/1
* 1/2
* 1/4
* 1/8
* 1/16
* 1/32
* 1/64
* 1/128

These can be set with `set_pwm_control`:

```python
ioe.set_pwm_control(divider=8)
```

In order to dial in the frequency you need, you must consider the 24MHz clock, the available divider options and the maximum value of the period register.

For example, for a 50Hz servo frequency you would use a 1/8 divider, and a period of 60,000:

```
24,000,000 / 8 / 60,000 = 50
````

``python
ioe.set_pwm_control(divider=8)
ioe.set_pwm_period(60000)
```

Then you can use duty-cycle values from 3000 to 6000 (1ms to 2ms) to create a servo control pulse.