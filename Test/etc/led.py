import RPi.GPIO as gpio
from time import sleep

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)

red = 2
green = 3

gpio.setup(red, gpio.OUT)
gpio.setup(green, gpio.OUT)

try:
	while 1:
		gpio.output(red, gpio.HIGH)
		gpio.output(green, gpio.LOW)
		sleep(1)
		gpio.output(red, gpio.LOW)
		gpio.output(green, gpio.HIGH)
		sleep(1)

except KeyboardInterrupt:
	gpio.output(red, gpio.LOW)
	gpio.output(gree, gpio.LOW)
	gpio.cleanup()
