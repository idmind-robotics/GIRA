import update, env, lib.requests, lib.logger, lib.requests, lib.timew, time, os, machine
from lib import base64
import network

reprogram = False
t = lib.timew.Time(time=time)

try:
  # Check if the memory.txt file exists
  f = open('memory.txt')
  text = f.read()

  # If the file exists, check if it contains the word 'reprogram'
  if text == 'reprogram':
    # If it does, connect to WIFI, delete the file and set the reprogram flag to True
    wlan_ap = network.WLAN(network.AP_IF)
    wlan_sta = network.WLAN(network.STA_IF)
    wlan_sta.active(True)
    print('Trying to connect to %s...' % env.settings['wifiAP'])
    if not wlan_sta.isconnected():
        print('connecting to network...')
        wlan_sta.connect(env.settings['wifiAP'], env.settings['wifiPassword'])
        while not wlan_sta.isconnected():
            pass
    reprogram = True
    os.remove('memory.txt')
except Exception:
  print("no file present, no need to update")

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
updater = update.OTAUpdater(io=io, github=github, logger=loggerOta, machine=machine, localUpdate=not reprogram)

try:
  # Check for updates
    updater.update()
except Exception as e:
    log('Failed to OTA update:', e)

try:
  # Start the main program
  import src.main
  src.main.start(env=env, requests=lib.requests, logger=logger, time=t, updater=updater)
except Exception as e:
  # If the main program fails, reset the device
  print("FAILED", e)
  machine.reset()