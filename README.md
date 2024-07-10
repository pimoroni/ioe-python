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

## Installing

We'd recommend using this library with Raspberry Pi OS Bookworm or later. It requires Python ≥3.7.

### Full install (recommended):

We've created an easy installation script that will install all pre-requisites and get your IO Expander-based breakout
up and running with minimal efforts. To run it, fire up Terminal which you'll find in Menu -> Accessories -> Terminal
on your Raspberry Pi desktop, as illustrated below:

![Finding the terminal](http://get.pimoroni.com/resources/github-repo-terminal.png)

In the new terminal window type the command exactly as it appears below (check for typos) and follow the on-screen instructions:

```bash
git clone https://github.com/pimoroni/ioe-python
cd ioe-python
./install.sh
```

**Note** Libraries will be installed in the "pimoroni" virtual environment, you will need to activate it to run examples:

```
source ~/.virtualenvs/pimoroni/bin/activate
```

### Development:

If you want to contribute, or like living on the edge of your seat by having the latest code, you can install the development version like so:

```bash
git clone https://github.com/pimoroni/ioe-python
cd ioe-python
./install.sh --unstable
```

## Install stable library from PyPi and configure manually

* Set up a virtual environment: `python3 -m venv --system-site-packages $HOME/.virtualenvs/pimoroni`
* Switch to the virtual environment: `source ~/.virtualenvs/pimoroni/bin/activate`
* Install the library: `pip install pimoroni-ioexpander`

In some cases you may need to us `sudo` or install pip with: `sudo apt install python3-pip`.

This will not make any configuration changes, so you may also need to enable:

* i2c: `sudo raspi-config nonint do_i2c 0`

You can optionally run `sudo raspi-config` or the graphical Raspberry Pi Configuration UI to enable interfaces.

## Note for Raspberry Pi 1

The first version of the Raspberry Pi uses SMBus 0 instead of 1. The ioe-python library uses SMBus 1 by default.
You can change the SMBus that is used by adding `smbus_id=0` to your calls to `io.IOE(...)`. If you want your code to run on multiple revisions of the Pi without having to change your code depending on the Raspberry Pi revision, you can make your code check the `Revision` part of `/proc/cpuinfo` and set the SMBus accordingly. Revisions `0002` and `0003` use SMBus 0. All others use SMBus 1.

# Examples and Usage

There are various examples to get you started with your IO Expander. With the library installed on your Raspberry Pi, these can be found in the `~/Pimoroni/pimoroni-ioexpander/examples` directory.

To take IO Expander further, the full API is described in the [library reference](/REFERENCE.md), with additional feature specific information found in the [docs folder](/docs).

# Removing the Library

To uninstall the library only (keeping all examples):

* Just run `pip uninstall pimoroni-ioexpander`

Or if you have grabbed the library from Github:

* `cd ioe-python`
* `./uninstall.sh`
