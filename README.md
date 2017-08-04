# DW1000 Python Library for Raspberry Pi

This module allows the use of the DW1000 chip with a Raspberry Pi using Python scripts. It is an adaptation in Python language of the [arduino-dw1000] library.
It uses the monotonic module in the ranging scripts but if you have Python 3 installed on your Raspberry Pi, it is not required since you can use time.monotonic() in the time module.

[arduino-dw1000]: <https://github.com/thotro/arduino-dw1000>
[monotonic]: <https://github.com/atdt/monotonic>
