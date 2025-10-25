#!/bin/sh


"esptool --port /dev/ttyUSB0 --before default_reset --after hard_reset --no-stub run",

"esptool --port /dev/ttyUSB1 --before default_reset --after hard_reset --no-stub run"
 