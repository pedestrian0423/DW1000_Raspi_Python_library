# DW1000 Python Library for Raspberry Pi

This module provides functions allowing the use of the DW1000 chip with a Raspberry Pi using Python scripts. It is an adaptation in Python language of the [arduino-dw1000] library. 
This chip embeds Ultra Wide Band (UWB) technology which can be used for message transmissions and more commonly, ranging functionality.

The following external python modules/libraries are necessary for its proper use:
* [spidev] : This is a standard python module used to interface SPI devices with the Raspberry Pi via the spidev linux kernel driver.
* [monotonic] (used in the ranging scripts only) : This module provides a function returning the value of a clock which never goes backwards. If you have Python 3 installed on your Raspberry Pi, it is not required since you can use time.monotonic() instead.
* [RPi.GPIO] : This is the standard python module used to interact with the GPIOs available on the Raspberry Pi.

[arduino-dw1000]: <https://github.com/ThingType/arduino-dw1000>
[monotonic]: <https://github.com/atdt/monotonic>
[spidev]: <https://github.com/doceme/py-spidev>
[RPi.GPIO]: <https://sourceforge.net/p/raspberry-gpio-python/wiki/install/>
