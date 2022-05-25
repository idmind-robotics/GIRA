import time
from machine import Pin, ADC
import machine
import imu

class HW:
    def __init__(self):
        self.fivevenablePin = machine.Pin(25, Pin.OUT)
        self.gpsneablePin = machine.Pin(26, Pin.OUT)
        self.motorenablePin = machine.Pin(23, Pin.OUT)
        self.lockAPin = machine.Pin(19, Pin.OUT)
        self.lockBPin = machine.Pin(13, Pin.OUT)
        self.INT1Pin = machine.Pin(29, Pin.IN)
        self.INT2Pin = machine.Pin(15, Pin.IN)
        self.INT1Pin.irq(trigger=Pin.IRQ_RISING, handler=handle_interrupt_1)
        self.INT2Pin.irq(trigger=Pin.IRQ_RISING, handler=handle_interrupt_2)
        self.TSENSEPin = ADC(Pin(33))
        self.ISENSEPin = ADC(Pin(34))
        self.VSENSEPin = ADC(Pin(35))
        self.VSENSEPin.atten(ADC.ATTN_11DB)
        self.chargersensePin = ADC(Pin(32))
        self.chargersensePin.atten(ADC.ATTN_11DB)
        self.ISENSEPin.atten(ADC.ATTN_11DB)
        self.lockstatusPin = machine.Pin(16, Pin.IN)
        self.flagfivevoltsPin = machine.Pin(17, Pin.IN)
        self.flaggpsvoltsPin = machine.Pin(18, Pin.IN)
        self.buzzer = machine.PWM(machine.Pin(14), freq = 800, duty = 0)
        self.imu = imu.IMU()
        self.imu.setup_imu()
        self.motorsOn = False
        self.forceShutDown = False
        self.values={"OnDock":"", "ACCTilt":"", "ACCXX":"", "ACCYY":"", "ACCZZ":"", "VBat":"", "Temp":"", "IsLocked": "", "CBat": ""}
        self.turnOff()
        self.motorenablePin.value(0)
    
    def handle_interrupt_1(pin):
        print("Interrupt1")
    
    def handle_interrupt_2(pin):
        print("Interrupt1")
    
    def beep(self, _time = 0.05):
        self.buzzer.duty(300)
        time.sleep(_time)
        self.buzzer.duty(0)

    def unlock(self):
        self.lockAPin.value(1)
        self.lockBPin.value(0)
        time.sleep(0.1)
        self.lockAPin.value(0)

    def lock(self):
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
        return (9.66e-3*sensorValue+5.26)

    def calcChargerVoltage(self, sensorValue):
        return (9e-03*sensorValue+7.81)

    def calcCurrent(self, sensorValue):
        return (5.43e-03*sensorValue + 0.952)

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
        self.values["CBat"] = self.calcCurrent(self.ISENSE)
        self.values["VBat"] = self.calcBattVoltage(self.VSENSE)
        self.values["VCharger"] = self.calcChargerVoltage(self.chargersense)
        self.values["ACCXX"], self.values["ACCYY"], self.values["ACCZZ"] = self.imu.read()
        self.values["IsLocked"] = self.lockstatus
        #CHECK IF WE'RE DOCKED SO WE TURN THE MOTORS OFF
        if self.values["VCharger"]  > 20:
            self.values["OnDock"] = 1
            self.turnOffMotors()
        else:
            self.values["OnDock"] = 0
        #CHECK IF BAT VALUE IS LOW SO WE TURN THE MOTORS OFF
        # print(self.values)
        # print(self.values["VBat"], self.motorsOn, self.values["OnDock"])
        if self.values["VBat"] < valor*.2:
            self.turnOffMotors()
        elif self.values["VBat"] > valor*.25 and not self.motorsOn and self.values["OnDock"] == 0:
            self.turnOnMotors()
        
