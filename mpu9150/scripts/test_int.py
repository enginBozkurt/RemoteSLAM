#!/usr/bin/env python

import pigpio
from timeit import default_timer as timer
import time

def cbf(gpio, level, tick):
	print(gpio, level, tick)
	print(timer())
	

pi = pigpio.pi()

#pi.set_mode(24, pigpio.INPUT)
#pi.write(24, 0)
#pi.set_pull_up_down(24, pigpio.PUD_DOWN)

pi.set_mode(25, pigpio.INPUT)
pi.write(25, 0)

cb1 = pi.callback(24, pigpio.RISING_EDGE, cbf)

while(True):
	pi.gpio_trigger(25, 100, 1)
	time.sleep(1)
