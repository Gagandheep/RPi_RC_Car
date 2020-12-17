import RPi.GPIO as GPIO
import curses
import os
from custom_bot import RobotTwoWheeled
import numpy as np


def main(win):
	GPIO.setmode(GPIO.BCM)
	bot = RobotTwoWheeled((26, 19), (20, 21), (23, 24))
	bot.speed_pid = 15

	win.nodelay(True)

	while True:

		try:
			key = win.getkey()
			os.system('clear')

			if key == '`':
				bot.command('f')
				break

			elif key == 'i':
				bot.move_steps(1, 1, 1)
				continue

			print(key)
			print(bot.command(key))
		except:
			pass


curses.wrapper(main)

