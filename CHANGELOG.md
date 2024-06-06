# Changelog

1.0.1
-----

* Fix bug in SuperIOE constructor (add i2c bus number)

1.0.0
-----

* Add dependency on smbus2
* Add support for alternate i2c bus number
* Port to hatch/pyproject.toml

0.0.5
-----

* Improved readme and documentation
* Linting fixes

0.0.4
-----

* Add support for bigger nuvoton chip
* Add wrappers for controlling motors, servos, and encoders

0.0.3
-----

* Add support for 7-bit switch/pulse counters

0.0.2
-----

* Call read/write in a single i2c_rdwr to maintain thread safety

0.0.1
-----

* Initial Release
