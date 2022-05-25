import time
import machine
from machine import Timer, SoftI2C
from machine import Pin, ADC


import imu
import gsm
import hw
import json

URL_BIKE_CONFIG="https://emel.city-platform.com/opendata/gira/bike/config"
URL_BIKE_TOKEN="https://emel.city-platform.com/opendata/data/gira/token"
URL_BIKE_API="https://emel.city-platform.com/opendata/gira/bike"
#"http://idmind-webdev-server.herokuapp.com/api/gps"
#
URL_BIKE_ID="http://idmind-webdev-server.herokuapp.com/api/bike_registry"
URL_BIKE_COMMANDS="http://idmind-webdev-server.herokuapp.com/api/command"

hw = hw.HW()
hw.turnOff()
time.sleep(1)

hw.turnOn()
hw.turnOnGps()
hw.turnOffMotors()

modem=gsm.Modem(MODEM_TX_PIN=2, MODEM_RX_PIN=4)
modem.initialize()
time.sleep(1)
modem.setupGSM()
time.sleep(5)
#get initial bike config
response = modem.http_request(URL_BIKE_ID, mode='POST', data=json.dumps({"IMEI":modem.values["IMEI"]}))
bike_id = json.loads(response.content)
print("BIKE_ID:")
print(bike_id)

CONTEXT = {
    "hw": hw,
    "modem": modem,
    "buzz": False,
    "lock": False,
    "unlock": False,
    "shutdown": False, 
    "turnon": False,
    "bike": bike_id,
    "hasBikeUpdate":False,
    "apiInfo": {},
    "lastBikeSent": time.time()
}

def hw_routine(t):
    if CONTEXT["shutdown"]:
        CONTEXT["hw"].forceMotorShutdown()
        CONTEXT["shutdown"] = False
    if CONTEXT["turnon"]:
        CONTEXT["hw"].forceMotorOn()
        CONTEXT["turnon"] = False
    if CONTEXT["buzz"]:
        CONTEXT["hw"].beep()
        CONTEXT["buzz"] = False
    if CONTEXT["lock"]:
        CONTEXT["hw"].lock()
        CONTEXT["lock"] = False
    if CONTEXT["unlock"]:
        CONTEXT["hw"].unlock()
        CONTEXT["unlock"] = False
    CONTEXT["hw"].read()

def comm_routine(t):
    start = time.time()
    bike_commands = CONTEXT["modem"].http_request(URL_BIKE_COMMANDS, mode='POST', data="{}")
    bike_commands = json.loads(bike_commands.content)
    if bool(bike_commands):
        if 'shutdown' in bike_commands.keys() and bike_commands["shutdown"]:
            CONTEXT["shutdown"] = True
        if 'turnon' in bike_commands.keys() and bike_commands["turnon"]:
            CONTEXT["turnon"] = True
        if 'buzz' in bike_commands.keys() and bike_commands["buzz"]:
            CONTEXT["buzz"] = True
        if 'lock' in bike_commands.keys() and bike_commands["lock"]:
            CONTEXT["lock"] = True
        if 'unlock' in bike_commands.keys() and bike_commands["unlock"]:
            CONTEXT["unlock"] = True
    end = time.time()
    print(end - start)

    start = time.time()
    CONTEXT["modem"].getGpsStatus()
    CONTEXT["modem"].getGpsData()
    end = time.time()
    print(end - start)
    final = {}
    final = CONTEXT["bike"].copy()
    final.update(CONTEXT["modem"].values)
    final.update(CONTEXT["hw"].values)
    print(final)
    start = time.time()
    bike_commands = CONTEXT["modem"].http_request(URL_BIKE_API, mode='POST', data=json.dumps(final))
    print(bike_commands.content)
    end = time.time()
    print(end - start)



hw_routine_timer = Timer(3)
hw_routine_timer.init(period=100, mode=Timer.PERIODIC, callback=hw_routine)

comm_routine_timer = Timer(1)
comm_routine_timer.init(period=1000, mode=Timer.PERIODIC, callback=comm_routine)

# comm_set_routine_timer = Timer(2)
# comm_set_routine_timer.init(period=10000, mode=Timer.PERIODIC, callback=comm_set_routine)

while True:
    time.sleep(1.0)
