# IO Expander

[![Build Status](https://travis-ci.com/pimoroni/ioe-python.svg?branch=master)](https://travis-ci.com/pimoroni/ioe-python)
[![Coverage Status](https://coveralls.io/repos/github/pimoroni/ioe-python/badge.svg?branch=master)](https://coveralls.io/github/pimoroni/ioe-python?branch=master)
[![PyPi Package](https://img.shields.io/pypi/v/pimoroni-ioexpander.svg)](https://pypi.python.org/pypi/pimoroni-ioexpander)
[![Python Versions](https://img.shields.io/pypi/pyversions/pimoroni-ioexpander.svg)](https://pypi.python.org/pypi/pimoroni-ioexpander)

IO Expander Breakout uses a Nuvoton MS51 microcontroller and I2C to give you 14 additional input/output pins to connect things up to. Eight of the pins are hooked up to an Analog to Digital Converter and six of the pins can be used as (up to 16-bit) PWM outputs.

This library is also used to power our other Nuvoton-based boards and breakouts!

## Where to buy

* IO Expander Breakout: https://shop.pimoroni.com/products/io-expander
* RGB Potentiometer Breakout: https://shop.pimoroni.com/products/rgb-potentiometer-breakout
* RGB Encoder Breakout: https://shop.pimoroni.com/products/rgb-encoder-breakout

# Installing

Stable library and dependencies from GitHub:

* `git clone https://github.com/pimoroni/ioe-python`
* `cd ioe-python`
* `sudo ./install.sh`

Latest/development library and dependencies from GitHub:

* `git clone https://github.com/pimoroni/ioe-python`
* `cd ioe-python`
* `sudo ./install.sh --unstable`

Stable (library only) from PyPi:

* Just run `pip3 install pimoroni-ioexpander`

In some cases you might need to use `sudo`.
