'''
 Main functioning loop of the program. Hardware and Modem are initiated and values read at set frequency.
 Reads and controls hardware devices and also posts all bike info periodically
'''

error_count = 0
import time
import machine
from machine import Timer, SoftI2C
from machine import Pin, ADC, UART
import sys
sys.path.append('/src')

from src import gsm
from src import hw
import json
import micropython
import select

hw = hw.HW()

'''
We want to check if we are connected to our configuration program to retrieve new bike_info. 
For that after resetting we will check for 15 seconds if we get a response.
'''

################## CHECK IF BIKE_ID FILE HAS ANY SAVED #################
try:    
    with open("bike_id.txt", "r+") as id_file:
        line = id_file.readline()
        if line:
            pass
        else:
            bike_id = '{"BIKERFID": "0", "BIKEREGISTRY": "0"}'
            id_file.write(str(bike_id))
except Exception as e:
    print(f"File opening error {e}")

def receive_message(timeout = 5):
    ''' Used to receive messages from the CLI App '''
    message = ""
    start = time.time()
    while time.time() - start < timeout:
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            # Assuming '\n' as the termination character
            if ch == '\n':
                return message
            else:
                message += ch

########    CHECK IF THE BATTERY IS CONNECTED TO THE CLI APP    #######
com = False
uart = UART(1, 115200, timeout = 500, rx=9, tx=10) # Verificar RX e TX 
start = time.time()
print("Checking if connected to ProgAPP")
while abs(time.time() - start) < 5:
    msg = receive_message()
    if msg is not None:
        if msg.strip() == "PC_CONNECTED":
            print("ESP_CONNECTED")
            com = True
            break
    time.sleep(1)

#######     IF CONNECTED TO CLI APP COM = TRUE: START LISTENING TO COMMANDS  ######
debug = False   # This will print stuff to the CLI app to try to find out whats wrong
while com:
    time.sleep(1)
    command = receive_message()
    
    if command == "config":
        # Check if there is a id_file.txt with info
        with open("bike_id.txt", "r+") as id_file:
            line = id_file.readline()
            print("Configuração Atual ->", end=" ")
            print(line)
            time.sleep(1)
        
        new_info = receive_message() # Keep waiting for the string containing the new bike-id
        while (new_info is not None and len(new_info) < 2) or new_info is None:
            new_info = receive_message()
            time.sleep(0.5)
            
        with open("bike_id.txt", "w+") as id_file:
            id_file.write(str(new_info))
            print(f"Nova Configuração Aplicada Com Sucesso: {new_info}")
    
    '''
    {'URL_BIKE_CONFIG': 'https://emel.city-platform.com/opendata/gira/bike/config','URLAPI': 'https://emel.city-platform.com/opendata/gira/bike', 'APN': 'umts' , 'user': '', 'passwd': ''}
    '''
    if command == "urlconf":
        new_conf = receive_message()
        while (new_conf is not None and len(new_conf) < 2) or new_conf is None:
            new_conf = receive_message()
            time.sleep(0.5)
            
        with open("url_conf.txt", "w+") as urls_file:
            urls_file.write(str(new_conf))
            print(f"Nova Configuração Aplicada com Sucesso: {new_conf}")
                            
    if command == "showconfig":
        # Check if there is a id_file.txt with info
        with open("bike_id.txt", "r+") as id_file:
            line = id_file.readline()
            print("Configuração Atual ->", end=" ")
            print(line)
            time.sleep(1)
        
    if command == "charge":
        hw.read()
        print(hw.values["BAT_VOLT"])
            
    if command == "debug": 
        com = False
        debug = True
            
    if command == "exit":
        com = False
    
    if command == "beep":
        beepPin = machine.Pin(26, Pin.OUT) # Turn on/off 4.2V regulator
        beepPin.value(1)
        buz = machine.PWM(machine.Pin(14), freq=3000, duty=32000)
        time.sleep(2)
        beepPin.value(0)
        buz.duty(0)
        buz.deinit()
            
#############       GET BIKEREGISTRY AND BIKERFID PARAMS FROM THE BIKE_ID FILE      #############
with open("bike_id.txt", "r+") as id_file:
        line = id_file.readline()
        bike_id = json.loads(str(line).replace("'", "\""))
print(f"Identificação da Bicicleta: {bike_id}")
if str(bike_id['BIKEREGISTRY']) == "0":
    print("Going to sleep")
    machine.deepsleep()
    
    
#############       GET URLAPI FROM URLS FILE      #############
with open("url_conf.txt", "r+") as urls_file:
        line = urls_file.readline()
        URLS = json.loads(str(line).replace("'", "\""))

#URLAPI = URLS['URLAPI']
URLAPI = "https://emel.city-platform.com/opendata/gira/bike" # The actual default URL to send data to

print("A iniciar Hardware...")
#hw = hw.HW() 	# Already declared on top
time.sleep(1)
hw.turnOn() #Turns ON 5V, 4.2V for GPS and MotorController output
hw.buzzer.buzzer.duty(0)
time.sleep(1)
print("Hardware Iniciado!")
hw.beep()



#############     SIM808 Modem initialization    #############
modem = gsm.Modem(MODEM_TX_PIN=2, MODEM_RX_PIN=4)
simINIT = False
#############    KEEP TRYING TO CONNECT TO THE SIM808 UNTIL AN IP IS OBTAINED  #############
while not simINIT:
    try:
        modem.initialize()
        time.sleep(5)
        success = modem.setupGSM()
        if success:
            simINIT = True
            print("Modem Iniciado com Sucesso!")
            hw.beep()
    except Exception as e:
        # Reboot the modem power and try again
        print(e)
        print("Inicialização do Modem falhada, a reiniciar e tentar novamente...")
        hw.turnOffGps()
        time.sleep(1)
        hw.turnOnGps()
        time.sleep(2)

####################################################


#############    GIRA API - Request initial configurations of the bike      #############
        
        
'''
URL_BIKE_CONFIG="https://emel.city-platform.com/opendata/gira/bike/config" # Obter a informação de configuração inicial (inclui tokenuser  tokenpassword)
response = modem.http_request(URL_BIKE_CONFIG, mode='POST', data="config")
configdata = json.loads(response.content)
#URLAPI = configdata["URLSYS"] # This should contain the same thing as URLAPI = "https://emel.city-platform.com/opendata/gira/bike"
URL_BIKE_COMMANDS = "https://emel.city-platform.com/opendata/gira/bike/config"
'''

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
    "msgTime" : 25, # How often to send bike data to the API, in seconds
    "errorCount": 0
}

error_count = 0
def hw_routine(t):
    try:
        if CONTEXT["shutdown"]:
            hw.forceMotorShutdown()
            CONTEXT["shutdown"] = False
        if CONTEXT["turnon"]:
            hw.forceMotorOn()
            CONTEXT["turnon"] = False
        if CONTEXT["buzz"]:
            hw.beep()
            CONTEXT["buzz"] = False
        if CONTEXT["lock"]:
            hw.lock()
            CONTEXT["lock"] = False
        if CONTEXT["unlock"]:
            hw.unlock()
            CONTEXT["unlock"] = False

        hw.read(modem.values["DateandTime"])  # Updates Hardware readings and handles control (Voltages, IMU, etc...) (hw.values)
        toprint = hw.values
        toprint.update(modem.values)
        print(toprint)
    
    except Exception as e:
        # If there's an error in hardware, write it to a file and keep going
        print(f"Erro HW {e}")
    
def comm_routine(t):
    try:
        '''
        # Request the data containing any sent commands through the web API (Lock, Unlock, Buzz, Shutdown, etc...)
        
        bike_commands = modem.http_request(URL_BIKE_COMMANDS, mode='POST', data="{}")
        bike_commands = json.loads(bike_commands.content) # Contains the Command Keys and Values of any sent commands

        if bool(bike_commands): # IF any commands were received
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
                f = open('memory.txt', 'w') # Reprogram command resets the board and updates through the GitHub repo
                f.write(' reprogram')  # The file is used to store the reprogram instruction through the reset
                f.close()
                machine.reset()
        '''
        
        try:
            modem.getGpsStatus() # Read Modem GPS and store GPSFix and GPSData in modem.values
            modem.getGpsData()
        except:
            print(" ")
            
        ##############      IF ENOUGH TIME HAS PASSED POST ALL DATA TO GIRA API       #############
        if time.time() - CONTEXT["lastBikeSent"] > CONTEXT["msgTime"]: # How often to post
            final = {'BIKETYPE': 'ELECTRICAL'}
            final.update(CONTEXT["bike"]) # Add Bike_ID
            final.update(modem.values) # Add ModemValues 
            final.update(hw.values) # Add HardwareValues
            final = json.dumps(final)
            print(final)
            URLAPI = "https://emel.city-platform.com/opendata/gira/bike"
            bike_commands = modem.http_request(URLAPI, mode='POST', data=final)
            #DEBUG_API = "https://webapps.robots.idmind.pt/gira/api/gps"
            #bike_commands = modem.http_request(DEBUG_API, mode='POST', data=final) #Send to DEBUG_API for checking
            CONTEXT["lastBikeSent"] = time.time()
            print("Dados Enviados para o Servidor com Sucesso!")
            
    except Exception as e:
        print(f"Erro COMUNICATION {e}")
        CONTEXT["errorCount"] += 1
        print(CONTEXT["errorCount"])
        if CONTEXT["errorCount"] >= 500:
            print("Resetting to solve connection error")
            machine.reset()
       
hw_routine_timer = Timer(3)
hw_routine_timer.init(period=100, mode=Timer.PERIODIC, callback=hw_routine) #Run the hardware routine 10x a second

comm_routine_timer = Timer(1)
comm_routine_timer.init(period=25000, mode=Timer.PERIODIC, callback=comm_routine) # Run the comms routine once every 20 seconds

while True:
    time.sleep(1)
