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
