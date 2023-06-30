import time
import machine
from machine import Timer, SoftI2C
from machine import Pin, ADC
from src import imu
from src import gsm
from src import hw
import json
import utime

URL_BIKE_CONFIG="https://emel.city-platform.com/opendata/gira/bike/config"
URL_BIKE_TOKEN="https://emel.city-platform.com/opendata/gira/token"
URL_BIKE_ID="https://webapps.robots.idmind.pt/gira/api/bike_registry"
URL_BIKE_COMMANDS="https://webapps.robots.idmind.pt/gira/api/command"

#DEBUG API
URL_BIKE_API="https://emel.city-platform.com/opendata/gira/bike"
DEBUG_API="https://webapps.robots.idmind.pt/gira/api/gps"


hw = hw.HW()
hw.turnOff()
time.sleep(1)

print("DEBUG: Turning hardware on")
hw.turnOn()
print("DEBUG: Turning GSM on")
hw.turnOnGps()
hw.turnOffMotors()
time.sleep_ms(200)
#initialize modem
modem=gsm.Modem(MODEM_TX_PIN=2, MODEM_RX_PIN=4)
modem.initialize()
hw.beep()
time.sleep(1)
#setup modem
modem.setupGSM()
hw.beep()
hw.beep()
# time.sleep(5)
#get initial bike config. This will be replaced eventually when EMEL provides a proper API
response = modem.http_request(URL_BIKE_ID, mode='POST', data=json.dumps({"IMEI":modem.values["IMEI"]}))
bike_id = json.loads(response.content)

#Service to return bike configuration information
response = modem.http_request(URL_BIKE_CONFIG, mode='POST', data=json.dumps({}))
bike_config = json.loads(response.content) 
#authenticate with the emel server
response = modem.http_request(URL_BIKE_TOKEN, mode='POST', data=json.dumps({"grant_type":"password","username":bike_config["TOKENUSR"],"password":bike_config["TOKENPASS"]}))
#get access token
security_config = json.loads(response.content)
token = security_config["access_token"]

#flag to change the api endpoint
sendToDebug = False

#initialize CONTEXT object
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
    "msgTime" : 10,
    "TOKENPASS": bike_config["TOKENPASS"],
    "TOKENUSR": bike_config["TOKENUSR"],
    "APNNAME": bike_config["APNNAME"],
    "APNUSR": bike_config["APNUSR"],
    "APNPASS": bike_config["APNPASS"],
    "sendToDebug": sendToDebug,
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
    #get commands from bike
    bike_commands = CONTEXT["modem"].http_request(URL_BIKE_COMMANDS, mode='POST', data="{}")
    print(bike_commands.content)
    bike_commands = json.loads(bike_commands.content)

    #for each command, update CONTEXT
    if bool(bike_commands):
        if 'shutdown' in bike_commands.keys() and bike_commands["shutdown"]:
            CONTEXT["shutdown"] = True
        if 'turnon' in bike_commands.keys() and bike_commands["turnon"]:
            CONTEXT["turnon"] = True
        if 'buzz' in bike_commands.keys() and bike_commands["buzz"]:
            CONTEXT["hw"].beep()
        if 'lock' in bike_commands.keys() and bike_commands["lock"]:
            CONTEXT["hw"].lock()
        if 'unlock' in bike_commands.keys() and bike_commands["unlock"]:
            CONTEXT["hw"].unlock()
        if 'debug' in bike_commands.keys() and bike_commands["debug"]:
            CONTEXT["debug"] = True
            machine.reset()
        if 'reprogram' in bike_commands.keys() and bike_commands["reprogram"]:
            #flag to allow reprogramming on reboot
            f = open('memory.txt', 'w')
            f.write('reprogram')
            f.close()
            machine.reset()

    CONTEXT["modem"].getGpsStatus()
    # utime.sleep_ms(10)
    CONTEXT["modem"].getGpsData()
    # utime.sleep_ms(10)
    #OBEY TO THE SPECIFIED COMMUNICATION RATE
    if time.time() - CONTEXT["lastBikeSent"] > CONTEXT["msgTime"]:
        final = {}
        final = CONTEXT["bike"].copy()
        if CONTEXT["modem"].values["GPSFix"] == "Location not Fix":
            print("GPS not fix")
            CONTEXT["gpsFix"] = False
        else:
            CONTEXT["gpsFix"] = True
            final.update(CONTEXT["modem"].values)

        final.update(CONTEXT["hw"].values)
        CONTEXT["lastBikeSent"] = time.time()
        print("DEBUG: bike status -> {}".format(json.dumps(final)))
        #SEND ANY COMMANDS TO THE BIKE
        if CONTEXT["sendToDebug"]:
            bike_commands = CONTEXT["modem"].http_request(DEBUG_API, mode='POST', data=json.dumps(final))
            CONTEXT["sendToDebug"] = False
        else:
            bike_commands = CONTEXT["modem"].http_request(URL_BIKE_API, mode='POST', data=json.dumps(final))
            CONTEXT["sendToDebug"] = True
        
hw_routine_timer = Timer(3)
hw_routine_timer.init(period=100, mode=Timer.PERIODIC, callback=hw_routine)

comm_routine_timer = Timer(1)
comm_routine_timer.init(period=1000, mode=Timer.PERIODIC, callback=comm_routine)

while True:
    time.sleep(1.0)