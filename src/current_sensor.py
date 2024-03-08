#! /usr/bin/env python
'''
Micropython class to interface with ACS37800 AC/DC Power Monitor through I2C
When ran as standalone the script checks the GIRA Power/Control Electronic Board for connectivity with the
2 sensors and allows for checking if BatteryVoltage, MotorCurrent, ChargerVoltage and ChargerCurrent readings are working
'''

from machine import Pin, SoftI2C
import time

MOTOR_SENSOR_ADDR = 0x6c # Motor Current Sensor
CHARGING_SENSOR_ADDR = 0x63 # Charging Input Current Sensor CAREFUL REVERSED FROM EAGLE

enable_time = 0

def bytes2binstr(b, n=None):
    s = ' '.join(f'{x:08b}' for x in b)
    return s if n is None else s[:n + n // 8 + (0 if n % 8 else -1)]

def set_bit(byte_array, bit_position):
    byte_position = bit_position // 8
    bit_offset = bit_position % 8
    byte_array[byte_position] |= 1 << bit_offset


'''
If AC bypass_n_en = 0 (default)
If DC bypass_n_en = 1
Sense and Divider Resistance must be set according to actual setup
'''


class PowerSensor:
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.device_addr = addr 
        self.voltage = 0
        self.current = 0
        self.power = 0
        self.dividerResistance = 2000000 # Default values for recommended DC Application
        self.senseResistor = 8200
        self.currentRange = 90 # Must be set according to sensor version (30A or 90A)

    def setCurrentRange(self, currentRange):
        self.currentRange = currentRange

    def setSenseResistor(self, senseResistor):
        self.senseResistor = senseResistor

    def setDividerResistance(self, dividerResistance):
        self.dividerResistance = dividerResistance
        
    def sendCustomerCode(self):
        # Customer code must be sent to the sensor before writting to any register (shadow or eeprom)
        customer_access_code =  0x4F70656E
        data_bytes = bytearray([customer_access_code & 0xFF, (customer_access_code >> 8) & 0xFF, (customer_access_code >> 16) & 0xFF, customer_access_code >> 24])
        self.i2c.writeto_mem(self.device_addr, 0x2F, data_bytes)
        time.sleep(1)
        #Check if it was accepted
        access = self.i2c.readfrom_mem(self.device_addr, 0x30, 1)
        binary_representation =  bytes2binstr(access)
        if binary_representation[-1] == "1":
            #print("Access permitted read from 0x30: True")
            pass
        else:
            print("Trouble Communicating with Voltage/Current Sensors")


    def setup(self, DC = True):
        '''
        All writes will be made to shadow memory at startup instead of stored in 
        EEPROM so we dont accidentaly brick the device
        The registers are locked by default. To unlock, write the customer access code 0x4F70656E to volatile regsiter 2F.
        Register 2F is write-only. You cannot read the access code back again.
        Register 30 will change from 0 to 1 if the code has been accepted.
        You need to cycle the power to disable customer write access. Writing zero to register 2F does not disable write access.
        '''
        self.DC = DC
        # Setup power sensor for DC application:
        if self.DC:
            # Start the setup by sending the customer code and veryfying if it was accepted
            self.sendCustomerCode()
            
            # Set number of samples for RMS calculations to 1023 (máximo nr de amostras) 0b1111111111 on bits 23 to 14
            data = self.i2c.readfrom_mem(self.device_addr, 0x1F, 4)
            byte = bytearray(data) # Read 4 bytes
            
            # Set bits 14 to 23 to 1
            mask_range = ((1 << 10) - 1) << 14
            value = (byte[3] << 24) | (byte[2] << 16) | (byte[1] << 8) | byte[0]
            value |= mask_range

            byte[0] = (value >> 0) & 0xFF
            byte[1] = (value >> 8) & 0xFF
            byte[2] = (value >> 16) & 0xFF
            byte[3] = (value >> 24) & 0xFF
            
            # Write the modified bytes back to the device
            self.sendCustomerCode()
            self.i2c.writeto_mem(self.device_addr, 0x1F, byte)
            time.sleep(0.1)
            
            # Fixed setting of N - we must change bit 24 of 0x1F address from 0 (default) to 1
            data = self.i2c.readfrom_mem(self.device_addr, 0x1F, 4)
            byte = bytearray(data) # Read 4 bytes
            bit_to_set = 24
            set_bit(byte, bit_to_set)
            self.sendCustomerCode()
            self.i2c.writeto_mem(self.device_addr, 0x1F, byte) #Change the bit to 1    
            time.sleep(0.1)

            
        else: # If sensor for AC application, leave as default
            pass

    def read(self):
        '''
        Instant voltage and current measurements read from reg 0x2A and power from reg 0x2C
        '''
        INSTANT_MEAS_REG = 0x2A
        data = self.i2c.readfrom_mem(self.device_addr, INSTANT_MEAS_REG, 4) # Read the 4 bytes
        
        # Extract vcodes as signed int
        signed_unsigned_vcodes = (data[1] << 8) | data[0]
        
        # Check if the sign bit is set (16th bit)
        if signed_unsigned_vcodes & 0x8000:
            # Negative value, convert using two's complement
            signed_vcodes = signed_unsigned_vcodes - 0x10000
        else:
            # Positive value
            signed_vcodes = signed_unsigned_vcodes
        
        volts = float(signed_vcodes)
        
        # Convert from codes to the fraction of ADC Full Scale
        volts /= 27500.0

        # Convert to mV (Differential Input Range is +/- 250mV)
        volts *= 250

        # Convert to Volts
        volts /= 1000

        # Voltage IN to Voltage LINE conversion
        self.voltage = volts * (self.dividerResistance + self.senseResistor) / self.senseResistor
        self.voltage = round(self.voltage, 1)
        
        # Extract the icodes as signed int
        signed_unsigned_icodes = (data[3] << 8) | data[2]
        
        # Check if the sign bit is set (16th bit)
        if signed_unsigned_icodes & 0x8000:
            # Negative value, convert using two's complement
            signed_icodes = signed_unsigned_icodes - 0x10000
        else:
            # Positive value
            signed_icodes = signed_unsigned_icodes
         
        # Convert icodes to current in Amps
        amps = float(signed_icodes)
        
        amps /= 27500.0
        amps /= 3
        if (amps * self.currentRange) < self.currentRange:
            self.current = amps * self.currentRange
            self.current = round(self.current, 1)
        
        # Read the instantaneous power from reg 0x2C and convert to mVAR
        INSTANT_POWER_REG = 0x2C
        data = self.i2c.readfrom_mem(self.device_addr, INSTANT_POWER_REG, 4) # Read the 4 bytes
        
        # Extract pinstant as signed int. Convert to W
        signed_unsigned_pinstant = data[1] << 8 | data[0]
        
        if signed_unsigned_pinstant & 0x8000:
            #Value is negative
            signed_pinstant  = signed_unsigned_pinstant - 0x10000
        else: #Value is positive
            signed_pinstant = signed_unsigned_pinstant
            
        self.power = float(signed_pinstant)
        # Datasheet says: 3.08 LSB/mW for the 30A version and 1.03 LSB/mW for the 90A version
        if self.currentRange == 30:
            lsb_per_mw = 3.08 * (30.0 / self.currentRange)
        else:
            lsb_per_mw = 1.03 * (30.0 / self.currentRange)  # Correct for sensor version
        
        self.power /= lsb_per_mw
        
        resistor_multiplier = (self.dividerResistance + self.senseResistor) / self.senseResistor
        self.power *= resistor_multiplier
        
        # Convert from mW to W
        self.power /= 1000

        return abs(self.voltage), abs(self.current), abs(self.voltage * self.current)
    
    
        
# ------------- TESTING PART --------------- #
'''
motor_enable = Pin(23, Pin.OUT)
motor_enable.value(1)
charger_enable = Pin(32, Pin.OUT)
charger_enable.value(0)

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq = 200000)
devices = i2c.scan()

if len(devices) == 0:
    print("No i2c device !")
else:
    print('i2c devices found:', len(devices))
    for device in devices:  
        print("Decimal address: ", device, " | Hexa address: ",hex(device))
        
if len(devices) == 2:
    MOTOR_SENSOR_ADDR = 0x6C # Battery Voltage and Motor Current Sensor
    CHARGING_SENSOR_ADDR = 0x63 # Charging Voltage and Current Sensor 
    motorSensor = PowerSensor(i2c, MOTOR_SENSOR_ADDR)
    chargeSensor = PowerSensor(i2c, CHARGING_SENSOR_ADDR)
    print("Declaration worked")
    motorSensor.setup()
    chargeSensor.setup()
    print("Setup Worked ")


    while True:
        battery_voltage, motor_current, motor_power = motorSensor.read()
        charger_voltage, charger_current, charger_power = chargeSensor.read()
        print(f"Battery Voltage: {battery_voltage}, Motor Current: {motor_current}, Charger Voltage: {charger_voltage}, Charger Current: {charger_current} ")
        
        if charger_voltage > 10: 
            print("CHARGER DETECTED")
            time.sleep(2)
            charger_enable.value(1)
            motor_enable.value(0)
        else:
            motor_enable.value(1)
            charger_enable.value(0)
            
        if battery_voltage >= 41.8:
            print("FULL BATTERY")
            charger_enable.value(0)
            time.sleep(1)
            
        time.sleep(1)
'''
            


