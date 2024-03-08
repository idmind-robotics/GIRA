'''
Script to run once at boot. Checks a memory.txt file to know to update from GitHub or locally.
After checking for updates, starts up the main loop of the program "main.py".
'''

import update, machine, env, lib.requests, lib.logger, lib.requests, lib.timew, time, os
from lib import base64
import network
import sys
sys.path.append('/src')
machine.freq(240000000)

reprogram = False
t = lib.timew.Time(time=time)

try:
  # Check if the memory.txt file exists: This file is created when a REPROGRAM command is received on the main loop
  f = open('memory.txt')
  text = f.read()

  # If the file exists, check if it contains the word 'reprogram'
  if text == 'reprogram': # If it does, connect to WIFI, delete the file and set the reprogram flag to True
    # TODO Try to use the Modem internet connection for retrieving updates from GitHub 
    wlan_ap = network.WLAN(network.AP_IF) # For setting up ESP as an AccessPoint
    wlan_sta = network.WLAN(network.STA_IF) # For connecting ESP to a Router
    wlan_sta.active(True)
    print('Trying to connect to %s...' % env.settings['wifiAP']) 
    if not wlan_sta.isconnected(): # If not connected
        print('connecting to network...')
        wlan_sta.connect(env.settings['wifiAP'], env.settings['wifiPassword']) # Connect ESP32 to WiFi 
        while not wlan_sta.isconnected(): # Wait until connection is established
            pass
    reprogram = True
    os.remove('memory.txt') # Delete the memory file so that it doesnt reprogram again next boot
except Exception:
  print("No memory file exists, no need to update")

# Configure Logger
logger = lib.logger.config(enabled=True, include=env.settings['logInclude'], exclude=env.settings['logExclude'], time=t)
log = logger(append='boot')
log("The current time is %s" % t.human())

loggerOta = logger(append='OTAUpdater')

# Configure OTA Updater
io = update.IO(os=os, logger=loggerOta)
github = update.GitHub(
    io=io,
    remote=env.settings['githubRemote'],
    branch=env.settings['githubRemoteBranch'],
    logger=loggerOta,
    requests=lib.requests,
    username=env.settings['githubUsername'],
    token=env.settings['githubToken'],
    base64=base64,
)

# Configure OTA Updater
updater = update.OTAUpdater(io=io, github=github, logger=loggerOta, machine=machine, localUpdate = not reprogram)

try:
    # Check for updates
    updater.update() # If reprogram is True, it will check GitHub repo. If not, it will just replace the src directory with the Aux directory (Why??)
except Exception as e:
    log('Failed to OTA update:', e)

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