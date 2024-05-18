#!/usr/bin/env python3

import time
from connector import BoardConnector

bot = BoardConnector(com="/dev/ttyUSB0")
bot.create_receive_threading()
time.sleep(1)

bot.set_motor(40, 0, 0, 0)
for i in range(10):
    print(bot.get_motor_encoder())
    time.sleep(1)

bot.set_motor(0, 0, 0, 0)
time.sleep(2)

bot.set_motor(-40, 0, 0, 0)
for i in range(10):
    print(bot.get_motor_encoder())
    time.sleep(1)

bot.set_motor(0, 0, 0, 0)
time.sleep(2)
