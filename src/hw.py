import time
from machine import Pin, ADC, SoftI2C
import machine
import json
from src import imu, current_sensor, buzzer
import math

class HW:
    def __init__(self):
        
        self.buzzer = buzzer.buzzer()
        self.buzzer.buzzer.duty(0)

        
        '''
        Define GPIO pins
        '''
        self.fiveEnablePin = machine.Pin(25, Pin.OUT) # Turn on/off 5V regulator
        self.gpsEnablePin = machine.Pin(26, Pin.OUT) # Turn on/off 4.2V regulator
        self.motorEnablePin = machine.Pin(23, Pin.OUT) # Switch on/off mosfet for allowing motors to run or not
        self.chargeEnablePin = machine.Pin(32, Pin.OUT) # Switch on/off mosfet for allowing charger current
        self.lockAPin = machine.Pin(19, Pin.OUT) 
        self.lockBPin = machine.Pin(13, Pin.OUT)
        self.lockStatusPin = machine.Pin(16, Pin.IN) # Reports back lock status
        self.flagFiveVoltsPin = machine.Pin(17, Pin.IN) # Reports back 5V regulator proper operation or not
        self.flagGpsVoltsPin = machine.Pin(18, Pin.IN) # Reports back 4.2V regulator proper operation or not
        self.TSENSEPin = ADC(Pin(33)) # Connected to termistor Voltage Divider with voltage buffer
        self.TSENSEPin.atten(ADC.ATTN_11DB)
        
        '''
        Define auxiliary variables for buzzer and alarm timings
        '''
        self.shakeAlarmStartTime = 0
        self.shakeAlarmStartFlag = False
        self.batteryAlarmFlag = False
        self.shakeAlarmDuration = 10 # After shaking stops, how long to keep the alarm ON
        self.BatVoltage100 = 42 # Voltage corresponding to a 100% Charge
        self.CutOffVoltage = 33.5 # Voltage corresponding to a ~30% Charge
        self.LowBattery = False
        
        '''
        Startup I2C Devices
        IMU and 2 Current Sensors
        '''
        self.SCL = Pin(22, Pin.OUT)
        self.SDA = Pin(21, Pin.OUT)
        self.i2c = SoftI2C(self.SCL, self.SDA)
        devices = self.i2c.scan()
        if len(devices) == 0:
            print("No i2c device found, should find 3! BoardFailure")
        else:
            print('Should find 3 devices, found:',len(devices))
            for device in devices:  
                print("Decimal address: ", device, " | Hexa address: ",hex(device))

        self.imu = imu.IMU(self.i2c)
        self.shakeDetector = imu.shakeDetection(window_size = 15, threshold = 1.3)
        self.tiltDetector = imu.tiltDetection(tilt_threshold = 80, time_threshold = 5) #Angle, time in seconds

        MOTOR_SENSOR_ADDR = 0x6C # Motor Current Sensor
        CHARGING_SENSOR_ADDR = 0x63 # Charging Input Current Sensor 
        self.motorSensor = current_sensor.PowerSensor(self.i2c, MOTOR_SENSOR_ADDR)
        self.motorSensor.setup()
        self.chargeSensor = current_sensor.PowerSensor(self.i2c, CHARGING_SENSOR_ADDR)
        self.chargeSensor.setup()

        '''
        Initialize remaining variables
        '''
        self.motorsOn = False
        self.chargerOn = False
        self.forceShutDown = False
        self.values={"OnDock": False, "Temp": -1, "IsLocked": False,
                    "ACCXX": 0 , "ACCYY": 0, "ACCZZ": 0, "Shaking": False, "IsTilted": False, "TiltAngle": 0,
                    "ChargerVoltage": 0, "ChargerCurrent": 0, "ChargerPower": 0, "ChargerEnabled": False,
                    "VBat": 0, "MotorCurrent": 0, "MotorPower": 0, "MotorEnabled": False}
    
        self.turnOff()
        self.forceMotorOn()
    
    def beep(self):
        self.buzzer.beeper()

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
        '''
        Turns OFF All 5V devices, GPS and Motors
        '''
        self.fiveEnablePin.value(0)
        self.gpsEnablePin.value(0)
        self.motorEnablePin.value(0)

    def turnOn(self):
        '''
        Turns ON All 5V devices, GPS and Motors
        '''
        if not self.LowBattery:
            self.turnOn5V()
            self.turnOnGps()
            self.turnOnMotors()

    def turnOn5V(self):
        self.fiveEnablePin.value(1)  

    def turnOnGps(self):
        self.gpsEnablePin.value(1)
        print("Alimentação GPS Ligada")

    def turnOff5V(self):
        self.fiveEnablePin.value(0)  

    def turnOffGps(self):
        self.gpsEnablePin.value(0)

    def turnOnMotors(self):
        if self.forceShutDown:
            return
        if not self.motorsOn and not self.LowBattery:
            self.motorEnablePin.value(1)
            self.motorsOn = True
            print("Alimentação Controlador Ligada")

    def turnOffMotors(self):
        if self.motorsOn:
            self.motorEnablePin.value(0)
            self.motorsOn = False
    
    def forceMotorShutdown(self):
        self.forceShutDown = True
        self.turnOffMotors()

    def forceMotorOn(self):
        if not self.LowBattery:
            self.forceShutDown = False
            self.turnOnMotors()

    def turnOnCharger(self):
        if not self.chargerOn:
            self.chargeEnablePin.value(1)
            self.chargerOn = True
            self.values["ChargerEnabled"] = True
    
    def turnOffCharger(self):
        if self.chargerOn:
            self.chargeEnablePin.value(0)
            self.chargerOn = False
            self.values["ChargerEnabled"] = False

    def termistorConversion(self, adcReading):
        # Steinhart-Hart Model Coefficients https://rusefi.com/Steinhart-Hart.html
        A = 0.006960076882193165
        B = -0.00045345680345257013 
        C = 7.88165834289982e-7
        #Turn ADC Reading into Voltage reading
        voltage_in = adcReading * 3.3 / 4096
        # Turn the Voltage Reading into the NTC resistance
        ntc = (3.3 * 10000 / voltage_in) - 10000
        # Apply Steinhart-Hart model 
        inv_temperature = A + B * math.log(ntc) + C * (math.log(ntc))**3 # In Kelvin
        return round(1/inv_temperature - 273.15 - 3) # -3 is an empyrical offset

    def read(self, GNStime=time.localtime()):
        '''
        This is the function to be called on the functioning loop 
        to continuously monitor the state of the system
        and update all measurements to the hw.values dictionary.
        '''        
        self.TSENSE = self.TSENSEPin.read()
        self.lockStatus = self.lockStatusPin.value()
        self.flagfivevolts = self.flagFiveVoltsPin.value()
        self.flaggpsvolts = self.flagGpsVoltsPin.value()
        
        self.values["Temp"] = self.termistorConversion(self.TSENSE)
        self.values["ACCXX"], self.values["ACCYY"], self.values["ACCZZ"] = self.imu.acceleration
        self.values["Shaking"] = self.shakeDetector.detect(self.values["ACCXX"], self.values["ACCYY"], self.values["ACCZZ"])
        self.values["TiltAngle"], self.values["IsTilted"] = self.tiltDetector.detect(self.values["ACCXX"], self.values["ACCYY"], self.values["ACCZZ"])
        self.values["ChargerVoltage"], self.values["ChargerCurrent"], self.values["ChargerPower"] = self.chargeSensor.read()
        self.values["VBat"], self.values["MotorCurrent"], self.values["MotorPower"] = self.motorSensor.read()
        self.values["IsLocked"] = bool(self.lockStatus)
        self.values["MotorEnabled"] = bool(self.motorEnablePin.value())
        self.values["ChargerEnabled"] = bool(self.chargeEnablePin.value())
        
        self.buzzer.alarm_check() # Check alarm start and timers
        
        '''
        Vandalism Detection while on Dock
        If heavy shaking motion is detected while the bike is docked, sound an alarm for fixed amount of time
        '''
        if (self.values["Shaking"] is True and self.values["OnDock"] is True and self.shakeAlarmStartFlag is False):
            self.buzzer.alarm = True
            self.shakeAlarmStartFlag = False
            self.shakeAlarmStartTime = time.time()
            
        if self.shakeAlarmStartFlag: # If the shake alarm is ON
            if (time.time() - self.shakeAlarmStartTime) >= self.shakeAlarmDuration: # If the alarm has been ON for the set amount of time, turn it OFF
                self.buzzer.alarm = False
     
        '''
        Tilt Detection and Alarm
        If the bike is detected to be laying on the ground for longer than the time_threshold set, sound an alarm (Flag only True if time has passed)
        '''
        if self.values["IsTilted"] is True and self.values["OnDock"] is False:
            self.buzzer.alarm = True
            
        if self.values["IsTilted"] is False:
            self.buzzer.alarm = False
            
        '''
        Dock detection 
        Monitor Read Charger Voltage to see if bike is docked or not. If bike is docked, Turn ON the charger
        '''
        if self.values["ChargerVoltage"] >= 30 and self.values["OnDock"] is False: # Charger Detected - Docked: Charger Voltage is expected to be 42 Volt
            self.values["OnDock"] = True
            self.forceMotorShutdown()
            self.turnOnCharger()
            
        elif self.values["ChargerVoltage"] <= 25 and self.values["OnDock"] is True: # Charger Disconnected - Undocked
            self.values["OnDock"] = False
            self.turnOffCharger()
            self.values["VBat"], self.values["MotorCurrent"], self.values["MotorPower"] = self.motorSensor.read()      
            if self.values["VBat"] > self.CutOffVoltage:
                self.forceMotorOn()
                self.turnOn5V()
            
        if self.values["OnDock"] is True and self.values["VBat"] >= 41.7 and self.values["ChargerEnabled"]:
            self.turnOffCharger()
            
        elif self.values["OnDock"] is True and self.values["VBat"] <= 41.3 and not self.values["ChargerEnabled"]:
            self.turnOnCharger()
            
        '''
        Battery Alarm
        If bike is riding and battery gets too low turn ON battery alarm and turn OFF motors
        '''
        if self.values["VBat"] <= self.CutOffVoltage and self.values["OnDock"] is False and self.batteryAlarmFlag is False: # Low Battery
            self.buzzer.alarm = True
            self.batteryAlarmFlag = True
            self.LowBattery = True
            self.turnOffMotors()
            self.turnOff5V()
            
        if (self.values["VBat"] > 34 or self.values["OnDock"] is True) and self.batteryAlarmFlag is True:
            self.buzzer.alarm = False
            self.batteryAlarmFlag = False
            self.LowBattery = False
    

        