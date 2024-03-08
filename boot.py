'''
Script to run once at boot. 
TODO: Check a memory.txt file to know to update from GitHub using SIM connection.
After checking for updates, starts up the main loop of the program "main.py".
'''

import update, machine, env, lib.requests, lib.logger, lib.requests, lib.timew, time, os
from lib import base64
import network
import sys
sys.path.append('/src')
machine.freq(240000000)
t = lib.timew.Time(time=time)

# This is done automatically - main.py (loop) is executed
'''
try:
  # Start the main program reading hardware and posting to API
  import main_loop
  main_loop.start(env=env, requests=lib.requests, logger=logger, time=t, updater=updater)
except Exception as e:
  # If the main program fails, reset the device
  print("FAILED to start main loop", e)
  machine.reset()
'''
