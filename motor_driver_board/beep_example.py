#!/usr/bin/env python3

import time
from connector import BoardConnector

bot = BoardConnector(com="/dev/ttyUSB0")

# The buzzer automatically beeps for 100 milliseconds before turning off
# Buzzer switch, on_time=0: off, on_time=1: keeps ringing
# On_time >=10: automatically closes after xx ms (on_time is a multiple of 10).
on_time = 100
bot.set_beep(on_time)
time.sleep(1)
