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


print("A iniciar Hardware...")
hw = hw.HW()
time.sleep(1)
hw.turnOn() #Turns ON 5V, 4.2V for GPS and MotorController output
hw.buzzer.buzzer.duty(0)
time.sleep(2)
print("Hardware Iniciado!")
hw.beep()



#############     SIM808 Modem initialization    #############
modem = gsm.Modem(MODEM_TX_PIN=2, MODEM_RX_PIN=4)
simINIT = False
#############    KEEP TRYING TO CONNECT TO THE SIM808 UNTIL AN IP IS OBTAINED  #############
while not simINIT:
    try:
        modem.initialize()
        time.sleep(1)
        success = modem.setupGSM()
        time.sleep(1)
        if success:
            simINIT = True
            print("Modem Iniciado com Sucesso!")
            hw.beep()
    except Exception as e:
        # Reboot the modem power and try again
        print("Inicialização do Modem falhada, a reiniciar e tentar novamente...")
        hw.turnOffGps()
        time.sleep(1)
        hw.turnOnGps()
        time.sleep(2)