import time
import machine
from machine import Timer, SoftI2C
from machine import Pin, ADC


from src import imu
from src import gsm
from src import hw
import json

URL_BIKE_CONFIG="https://emel.city-platform.com/opendata/gira/bike/config"
URL_BIKE_TOKEN="https://emel.city-platform.com/opendata/data/gira/token"
URL_BIKE_API="https://emel.city-platform.com/opendata/gira/bike"
DEBUG_API="http://idmind-webdev-server.herokuapp.com/api/gps"

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
    "lastBikeSent": time.time(),
    "debug": False, 
    "reprogram": False,
    "msgTime" : 80
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
    bike_commands = CONTEXT["modem"].http_request(URL_BIKE_COMMANDS, mode='POST', data="{}")
    print(bike_commands.content)
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
        if 'debug' in bike_commands.keys() and bike_commands["debug"]:
            CONTEXT["debug"] = True
            machine.reset()
        if 'reprogram' in bike_commands.keys() and bike_commands["reprogram"]:
            f = open('memory.txt', 'w')
            f.write('reprogram')
            f.close()
            machine.reset()

    CONTEXT["modem"].getGpsStatus()
    CONTEXT["modem"].getGpsData()
    
    if time.time() - CONTEXT["lastBikeSent"] > CONTEXT["msgTime"]:
        final = {}
        final = CONTEXT["bike"].copy()
        final.update(CONTEXT["modem"].values)
        final.update(CONTEXT["hw"].values)
        CONTEXT["lastBikeSent"] = time.time()
        # bike_commands = CONTEXT["modem"].http_request(URL_BIKE_API, mode='POST', data=json.dumps(final))
        # print(bike_commands.content)
        bike_commands = CONTEXT["modem"].http_request(DEBUG_API, mode='POST', data=json.dumps(final))
    


hw_routine_timer = Timer(3)
hw_routine_timer.init(period=100, mode=Timer.PERIODIC, callback=hw_routine)

comm_routine_timer = Timer(1)
comm_routine_timer.init(period=1000, mode=Timer.PERIODIC, callback=comm_routine)

# comm_set_routine_timer = Timer(2)
# comm_set_routine_timer.init(period=10000, mode=Timer.PERIODIC, callback=comm_set_routine)

while True:
    time.sleep(1.0)
