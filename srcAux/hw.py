from machine import Pin, ADC
import machine
import json
from src import imu
import math
import time

class HW:
    def __init__(self):
        #5V enable pin
        self.fivevenablePin = machine.Pin(25, Pin.OUT)
        self.gpsneablePin = machine.Pin(26, Pin.OUT)
        
        self.motorenablePin = machine.Pin(23, Pin.OUT)
        self.lockAPin = machine.Pin(19, Pin.OUT)
        self.lockBPin = machine.Pin(13, Pin.OUT)
        # self.INT1Pin = machine.Pin(5, Pin.IN)
        # self.INT2Pin = machine.Pin(15, Pin.IN)
        # self.INT1Pin.irq(trigger=Pin.IRQ_RISING, handler=self.handle_interrupt_1)
        # self.INT2Pin.irq(trigger=Pin.IRQ_RISING, handler=self.handle_interrupt_2)
        #tensao de carga
        self.TSENSEPin = ADC(Pin(33))
        #corrente de carga
        self.ISENSEPin = ADC(Pin(34))
        #voltagem de carga
        self.VSENSEPin = ADC(Pin(35))
        self.VSENSEPin.atten(ADC.ATTN_11DB)
        self.chargersensePin = ADC(Pin(32))
        #check https://docs.micropython.org/en/latest/esp32/quickref.html for more details
        self.chargersensePin.atten(ADC.ATTN_11DB)
        self.ISENSEPin.atten(ADC.ATTN_11DB)
        self.TSENSEPin.atten(ADC.ATTN_11DB)

        self.lockstatusPin = machine.Pin(16, Pin.IN)
        self.flagfivevoltsPin = machine.Pin(17, Pin.IN)
        self.flaggpsvoltsPin = machine.Pin(18, Pin.IN)
        self.buzzer = machine.PWM(machine.Pin(14), freq = 800, duty = 0)
        self.buzzer.duty(0)
        self.buzzer.deinit()
        # self.buzzer.deinit()
        self.imu = imu.IMU()
        self.imu.setup_imu()
        self.motorsOn = False
        self.forceShutDown = False
        self.values={"OnDock":"", "ACCTilt":"", "ACCXX":"", "ACCYY":"", "ACCZZ":"", "VBat":"", "Temp":"", "IsLocked": "", "CBat": "", "MotorOn" : "", "Shake" : ""}
        self.turnOff()
        self.motorenablePin.value(0)
        self.values["OnDock"] = 0
        self.lastBatVoltage = 0
        self.lastBatVoltage = self.calcBattVoltage(self.VSENSEPin.read())
        self.accx_array = []
        self.accy_array = []
        self.accz_array = []
    
    def beep(self, _time = 0.05):
        self.buzzer.init()
        self.buzzer.duty(300)
        time.sleep(_time)
        self.buzzer.duty(0)
        self.buzzer.deinit()
        

    def unlock(self):
        self.lockAPin.value(1)
        self.lockBPin.value(0)
        time.sleep(0.1)
        self.lockAPin.value(0)

    def lock(self):
        print("lock")
        self.lockAPin.value(0)
        self.lockBPin.value(1)
        time.sleep(0.1)
        self.lockBPin.value(0)

    def turnOff(self):
        self.fivevenablePin.value(0)
        self.gpsneablePin.value(0)

    def turnOn(self):
        self.fivevenablePin.value(1)
        self.gpsneablePin.value(1)

    def turnOn5V(self):
        self.fivevenablePin.value(1)  

    def turnOnGps(self):
        self.gpsneablePin.value(1)  

    def turnOff5V(self):
        self.fivevenablePin.value(0)  

    def turnOffGps(self):
        self.gpsneablePin.value(0)

    def turnOnMotors(self):
        if self.forceShutDown:
            return
        self.motorenablePin.value(1)
        self.motorsOn = True

    def turnOffMotors(self):
        if(self.motorsOn):
            self.motorenablePin.value(0)
        self.motorsOn = False
    
    def forceMotorShutdown(self):
        self.forceShutDown = True
        self.turnOffMotors()

    def forceMotorOn(self):
        self.forceShutDown = False

    def calcBattVoltage(self, sensorValue):
        #pode precisar de recalibracao com para as duas baterias diferentes
        voltage = -10.8 + 0.021*sensorValue + (-1.81e-6*sensorValue*sensorValue)
        return (voltage)

    def calcChargerVoltage(self, sensorValue):
        if sensorValue == 0:
            self.values["OnDock"] = 1
            self.turnOffMotors()
        elif sensorValue == 4095:
            self.values["OnDock"] = 0
        

    def calcCurrent(self, sensorValue):
        if sensorValue < 1000:
            return 0
        else:
            return (5.43e-03*sensorValue + 0.952)

    def calcShake(self, accx, accy, accz):
        self.accx_array.append(accx)
        self.accy_array.append(accy)
        self.accz_array.append(accz)
        if len(self.accx_array) > 10:
            self.accx_array.pop(0)
            self.accy_array.pop(0)
            self.accz_array.pop(0)

        nPositive = 0
        nNegative = 0
        for i in self.accx_array:
            if i >= 0:
                nPositive += 1
            else:
                nNegative += 1
        
        if nPositive > 2 and nNegative > 2:
            self.values["Shake"] = True
            print("DEBUG: Event -> Shake")
        else:
            self.values["Shake"] = False
        

    def read(self):
        #for tests lets keep val at 24V
        valor = 12
        self.TSENSE = self.TSENSEPin.read()
        self.VSENSE = self.VSENSEPin.read()
        self.ISENSE = self.ISENSEPin.read()
        self.chargersense = self.chargersensePin.read()
        self.lockstatus = self.lockstatusPin.value()
        self.flagfivevolts = self.flagfivevoltsPin.value()
        self.flaggpsvolts = self.flaggpsvoltsPin.value()
        # print("TSENSE:", self.TSENSE)
        # print("ISENSE:", self.calcCurrent(self.ISENSE))
        # print("VSENSE:", self.VSENSE)
        # print("chargersense:", self.chargersense)
        # print("lockstatus:", self.lockstatus)
        # print("flagfivevolts:", self.flagfivevolts)
        # print("flaggpsvolts:", self.flaggpsvolts)
        self.values["Temp"] = self.TSENSE
        self.values["CBat"] = self.calcCurrent(self.ISENSE)
        self.values["VBat"] = self.calcBattVoltage(self.VSENSE)
        self.calcChargerVoltage(self.chargersense)
        self.values["ACCXX"], self.values["ACCYY"], self.values["ACCZZ"] = self.imu.read()
        self.calcShake(self.values["ACCXX"], self.values["ACCYY"], self.values["ACCZZ"])
        self.values["IsLocked"] = self.lockstatus
        self.values["MotorOn"] = self.motorsOn
        
        if self.values["VBat"] < 32.3:
            self.turnOffMotors()
            self.turnOff5V()
        elif self.values["VBat"] > 34.5 and not self.motorsOn and self.values["OnDock"] == 0:
            self.turnOnMotors()
            self.turnOn5V()

        
