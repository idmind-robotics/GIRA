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
URL_BIKE_ID="http://idmind-webdev-server.herokuapp.com/api/bike_registry"
URL_BIKE_LOCK="http://idmind-webdev-server.herokuapp.com/api/command"

hw = hw.HW()
hw.turnOn()
hw.turnOnGps()

modem=gsm.Modem(MODEM_TX_PIN=2, MODEM_RX_PIN=4)
modem.initialize()
modem.setupGSM()
#get initial bike config
response = modem.http_request(URL_BIKE_ID, mode='POST', data=json.dumps({"IMEI":modem.values["IMEI"]}))
bike_id = json.loads(response.content)
print(bike_id)

CONTEXT = {
    "hw": hw,
    "modem": modem,
    "buzz": False,
    "lock": False,
    "unlock": False
}

def hw_routine(t):
    CONTEXT["hw"].read()
    if CONTEXT["buzz"]:
        CONTEXT["hw"].beep()
        CONTEXT["buzz"] = False
    if CONTEXT["lock"]:
        CONTEXT["hw"].lock()
        CONTEXT["lock"] = False
    if CONTEXT["unlock"]:
        CONTEXT["hw"].unlock()
        CONTEXT["unlock"] = False

def comm_get_routine(t):
    response = CONTEXT["modem"].http_request(URL_BIKE_LOCK, mode='POST', data="{}")
    response = json.loads(response.content)
    if bool(response):
        if 'lock' in response.keys() and response["lock"]:
            CONTEXT["lock"] = True
        if 'unlock' in response.keys() and response["unlock"]:
            CONTEXT["unlock"] = True



hw_routine_timer = Timer(3)
hw_routine_timer.init(period=100, mode=Timer.PERIODIC, callback=hw_routine)

comm_get_routine_timer = Timer(1)
comm_get_routine_timer.init(period=500, mode=Timer.PERIODIC, callback=comm_get_routine)

# comm_set_routine_timer = Timer(2)
# comm_set_routine_timer.init(period=10000, mode=Timer.PERIODIC, callback=comm_set_routine)

while True:
    time.sleep(1.0)
