#!/bin/bash
stm32flash -w out/radio-modem.hex -v -g 0x0 /dev/ttyUSB0
