#! /usr/bin/python3

from pynput import keyboard as kb

def pulsa(tecla):
	print('Se ha pulsado la tecla ' + str(tecla))

kb.Listener(pulsa).run()