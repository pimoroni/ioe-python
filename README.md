# IO Expander

[![Build Status](https://img.shields.io/github/actions/workflow/status/pimoroni/ioe-python/test.yml?branch=main)](https://github.com/pimoroni/ioe-python/actions/workflows/test.yml)
[![Coverage Status](https://coveralls.io/repos/github/pimoroni/ioe-python/badge.svg?branch=master)](https://coveralls.io/github/pimoroni/ioe-python?branch=master)
[![PyPi Package](https://img.shields.io/pypi/v/pimoroni-ioexpander.svg)](https://pypi.python.org/pypi/pimoroni-ioexpander)
[![Python Versions](https://img.shields.io/pypi/pyversions/pimoroni-ioexpander.svg)](https://pypi.python.org/pypi/pimoroni-ioexpander)

IO Expander Breakout uses a Nuvoton MS51 microcontroller and I2C to give you 14 additional input/output pins to connect things up to. Eight of the pins are hooked up to an Analog to Digital Converter and six of the pins can be used as (up to 16-bit) PWM outputs.

This library is also used to power our other Nuvoton-based boards and breakouts!

## Where to buy

### HATs

* Weather HAT: https://shop.pimoroni.com/products/weather-hat-only
* Inventor HAT Mini: https://shop.pimoroni.com/products/inventor-hat-mini


### Breakouts

* IO Expander Breakout: https://shop.pimoroni.com/products/io-expander
* RGB Potentiometer Breakout: https://shop.pimoroni.com/products/rgb-potentiometer-breakout
* RGB Encoder Breakout: https://shop.pimoroni.com/products/rgb-encoder-breakout
* MICS6814 3-in-1 Gas Sensor Breakout: https://shop.pimoroni.com/products/mics6814-gas-sensor-breakout


# Getting the Library

**Stable library only (no examples) from PyPi:**

* Just run `python3 -m pip install pimoroni-ioexpander`

In some cases you may need to install pip with: `sudo apt install python3-pip`

**Stable library, with latest examples from GitHub:**

* `git clone https://github.com/pimoroni/ioe-python`
* `cd ioe-python`
* `./install.sh`

**Latest/development library and examples from GitHub:**

* `git clone https://github.com/pimoroni/ioe-python`
* `cd ioe-python`
* `./install.sh --unstable`


# Configuring your Raspberry Pi

## Enable I2C

In order to use the IO Expander, you need to enable the I2C interface of your Raspberry Pi. This can be done in the terminal by running:

* `sudo raspi-config nonint do_i2c 0`

Alternatively, you can enable the I2C interface by:
* running `sudo raspi-config` and enabling the option under **Interfacing Options**.
* opening the graphical **Raspberry Pi Configuration** application from the **Preferences** menu.

You may need to reboot after enabling I2C for the change to take effect.

## Note for Raspberry Pi 1

The first version of the Raspberry Pi uses SMBus 0 instead of 1. The ioe-python library uses SMBus 1 by default.
You can change the SMBus that is used by adding `smbus_id=0` to your calls to `io.IOE(...)`. If you want your code to run on multiple revisions of the Pi without having to change your code depending on the Raspberry Pi revision, you can make your code check the `Revision` part of `/proc/cpuinfo` and set the SMBus accordingly. Revisions `0002` and `0003` use SMBus 0. All others use SMBus 1.

# Examples and Usage

There are various examples to get you started with your IO Expander. With the library installed on your Raspberry Pi, these can be found in the `~/Pimoroni/pimoroni-ioexpander/examples` directory.

To take IO Expander further, the full API is described in the [library reference](/REFERENCE.md), with additional feature specific information found in the [docs folder](/docs).


# Removing the Library

To uninstall the library only (keeping all examples):

* Just run `python3 -m pip uninstall pimoroni-ioexpander`

Or if you have grabbed the library from Github:

* `cd ioe-python`
* `./uninstall.sh`
