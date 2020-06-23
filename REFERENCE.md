# Pimoroni IO Expander

The Pimoroni IO Expander is based upon a Nuvoton MS51 and exposes much of the chip's functionality for reading/writing general IO, ADC and PWM.

- [Pimoroni IO Expander](#pimoroni-io-expander)
  - [Pins](#pins)
  - [Functions](#functions)
    - [General-purpose IO](#general-purpose-io)
      - [High-impedance input](#high-impedance-input)
      - [Input with pull-up](#input-with-pull-up)
      - [Output](#output)
    - [Analog Inputs (ADC)](#analog-inputs-adc)
    - [Pulse Width Modulation Outputs (PWM)](#pulse-width-modulation-outputs-pwm)
    - [Rotary Encoder Decoding](#rotary-encoder-decoding)
    - [Configuring Interrupts](#configuring-interrupts)

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

All pins support general-purpose IO and can be configured either as a high-impedance input, quasi-bidirectional input with pull-up, open-drain output or push-pull output. These modes are applicable to different use-cases and we mention some basic examples below.

#### High-impedance input

For input pins you will usually use `ioexpander.IN` which sets pins to input-only, high-impedance mode. This mode does not support pull-ups on the pins, and is useful for reading logic levels which are asserted to a high/low value.

To set a pin as a high-impedance input:

```python
ioe.set_mode(1, ioexpander.IN)
```

And read its value:

```python
value = ioe.input(1)
```

#### Input with pull-up

For reading buttons, or other inputs which sink the connected pin to ground (open drain IO pins for example) you should use the `ioexpander.IN_PU` mode.

In this mode the pin is set to a quasi-bidirectional input, and a pull-up resistor is asserted pulling the logic level weakly HIGH.

```python
ioe.set_mode(1, ioexpander.IN_PU)
```

Wire a button between ground, and the IO pin and read its value:

```python
value = ioe.input(1)
```

A value of 0 (`LOW`) corresponds to a pushed button.

#### Output

For output pins you may choose one of the following output modes:

* `ioexpander.PIN_MODE_PP` - Output, Push-Pull mode. Drives a pin either HIGH or LOW.
* `ioexpander.PIN_MODE_OD` - Output, Open-Drain mode. Drives low, or leaves the pin floating

Push-pull mode is non-inverting, and useful for controlling a connected device (such as a shift register or motor driver), or switching an NPN transistor.

Open-drain mode effectively inverts the signal, since outputting a HIGH will connect the pin to Ground. Open-drain outputs are used in multi-drop protocols like i2c, but can also be used for devices or digital logic that requires an active low input.

To set a pin as a push-pull output:

```python
ioe.set_mode(1, ioexpander.PIN_MODE_PP)
```

And set its value:

```python
ioe.output(1, 0)  # Low (Floating in OD)
ioe.output(1, 1)  # High (Low in OD)
```

Or an open-drain output:

```python
ioe.set_mode(1, ioexpander.PIN_MODE_OD)
```

And set its value:

```python
ioe.output(1, 0)  # High (High-impedance floating)
ioe.output(1, 1)  # Low (Pulls to ground)
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

```python
ioe.set_mode(1, ioexpander.PWM)
```

PWM outputs can optionally be inverted which us useful where you might be driving inverting buffers or common-cathode LEDs:

```python
ioe.set_mode(1, ioexpander.PWM, invert=True)
```

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

```python
ioe.set_pwm_control(divider=8)
ioe.set_pwm_period(60000)
```

Then you can use duty-cycle values from 3000 to 6000 (1ms to 2ms) to create a servo control pulse.

### Rotary Encoder Decoding

The IO Expander supports decoding the waveform from up to four rotary encoders. The A and B pins must be specified and are configured as schmitt trigger inputs with a pull-up, if the C pin is specified then it's set to open-drain and driven low. For example:

```python
ENC_CHANNEL = 1
POT_ENC_A = 12
POT_ENC_B = 3
POT_ENC_C = 11
ioe.setup_rotary_encoder(ENC_CHANNEL, POT_ENC_A, POT_ENC_B, pin_c=POT_ENC_C)
```

Each encoder channel has its own signed, 8bit count register which stores the continuous count of pulses as the encoder is rotated. This register is not reset between reads, and will overflow from 128 to -127 in one direction, and from 128 to -127 in the other.

In order to maintain a count across reads, this overflow event should be used to increment/decrement an offset which is then added to the register value. This is all done inside the IO Expander library, so you can simply read a continuous value using:

```python
count = ioe.read_rotary_encoder(1)
```

This value will correspond to the number of rotations of your rotary encoder dial, multiplied by the resolution of the encoder.

The rotary encoder channels will assert an interrupt when a value is changed, in your program main loop you should check for this interrupt, read the encoder value and clear the interrupt flag:

```
while True:
    if ioe.get_interrupt():
        count = ioe.read_rotary_encoder(1)
        ioe.clear_interrupt()
```

Note: in order to track overflows you will need to ensure this interrupt code can run fast enough to catch them. In most cases - ie: a person turning a dial with a 24 step resolution - even 1-second intervals are fine, but for decoding a motor you will want to sample much faster. For example a motor running at 20k RPM with a 12 step resolution would need to be sampled around 31 times a second or approximately every 30ms.

### Configuring Interrupts

IO Expander has an interrupt register to indicate a variety of state changes. On its own this interrupt register isn't much more useful than polling, but IO Expander can also generate an interrupt on its INT pin - connected to BCM 4 via Breakout Garden HAT - which you can then monitor with your GPIO library of choice.

By default the interrupt output pin is not used, but you can enable it on setup like so:

```python
import ioexpander

ioe = ioexpander.IOE(interrupt_pin=4)
```

In this instance `4` corresponds to `BCM4` on the Raspberry Pi. Specifying an interrupt pin will enable interrupt output on the IO Expander and set up `RPi.GPIO`.

Alternatively you can handle the interrupt how you see fit by initialising the library and enabling the interrupt output manually:

```python
import ioexpander

ioe = ioexpander.IOE()
ioe.enable_interrupt_out()
```

In either case the current state of the interrupt register (and pin) can be read by running:

```python
ioe.get_interrupt()
```

And cleared with:

```python
ioe.clear_interrupt()
```

If you're using the IO Expander library to handle interrupts then you can bind a handler to the interrupt event:

```python
import ioexpander

ioe = ioexpander.IOE(interrupt_pin=4)

def callback(channel):
    # Handle interrupt here
    ioe.clear_interrupt()

ioe.on_interrupt(callback)
```