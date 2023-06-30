import machine
from machine import Pin

#sound the buzzer to indicate the device is booting
buzzer = machine.PWM(machine.Pin(14), freq = 800, duty = 0)
buzzer.deinit()