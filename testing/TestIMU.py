class CBits:
    """
    Changes specific bits from a byte register
    """

    def __init__(
        self,
        num_bits: int,
        register_address: int,
        start_bit: int,
        register_width=1,
        lsb_first=True,
    ) -> None:
        self.bit_mask = ((1 << num_bits) - 1) << start_bit
        self.register = register_address
        self.star_bit = start_bit
        self.lenght = register_width
        self.lsb_first = lsb_first

    def __get__(
        self,
        obj,
        objtype=None,
    ) -> int:
        mem_value = obj._i2c.readfrom_mem(obj._address, self.register, self.lenght)

        reg = 0
        order = range(len(mem_value) - 1, -1, -1)
        if not self.lsb_first:
            order = reversed(order)
        for i in order:
            reg = (reg << 8) | mem_value[i]

        reg = (reg & self.bit_mask) >> self.star_bit

        return reg

    def __set__(self, obj, value: int) -> None:
        memory_value = obj._i2c.readfrom_mem(obj._address, self.register, self.lenght)

        reg = 0
        order = range(len(memory_value) - 1, -1, -1)
        if not self.lsb_first:
            order = range(0, len(memory_value))
        for i in order:
            reg = (reg << 8) | memory_value[i]
        reg &= ~self.bit_mask

        value <<= self.star_bit
        reg |= value
        reg = reg.to_bytes(self.lenght, "big")

        obj._i2c.writeto_mem(obj._address, self.register, reg)


class RegisterStruct:
    """
    Register Struct
    """

    def __init__(self, register_address: int, form: str) -> None:
        self.format = form
        self.register = register_address
        self.lenght = struct.calcsize(form)

    def __get__(
        self,
        obj,
        objtype=None,
    ):
        if self.lenght <= 2:
            value = struct.unpack(
                self.format,
                memoryview(
                    obj._i2c.readfrom_mem(obj._address, self.register, self.lenght)
                ),
            )[0]
        else:
            value = struct.unpack(
                self.format,
                memoryview(
                    obj._i2c.readfrom_mem(obj._address, self.register, self.lenght)
                ),
            )
        return value

    def __set__(self, obj, value):
        mem_value = value.to_bytes(self.lenght, "big")
        obj._i2c.writeto_mem(obj._address, self.register, mem_value)
        
from micropython import const
import time
from machine import Pin, SoftI2C
import machine
import math

try:
    from typing import Tuple
except ImportError:
    pass


_REG_WHOAMI = const(0x98)
_SENSOR_STATUS_REG = const(0x05)
_MODE_REG = const(0x07)
_ACC_RANGE = const(0x20)
_ACC_DATA_RATE = const(0x08)

# Acceleration Data
ACC_X_LSB = const(0x0D)
ACC_X_MSB = const(0x0E)
ACC_Y_LSB = const(0x0F)
ACC_Y_MSB = const(0x10)
ACC_Z_LSB = const(0x11)
ACC_Z_MSB = const(0x12)

# Sensor Power
STANDBY = const(0)
NORMAL = const(1)

# Acceleration Range
ACCEL_RANGE_2G = const(0b000)
ACCEL_RANGE_4G = const(0b001)
ACCEL_RANGE_8G = const(0b010)
ACCEL_RANGE_16G = const(0b011)
ACCEL_RANGE_12G = const(0b100)
accel_range_values = (
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G,
    ACCEL_RANGE_12G,
)

LPF_ENABLE = const(1)
LPF_DISABLE = const(0)

BANDWIDTH_1 = const(0b001)
BANDWIDTH_2 = const(0b010)
BANDWIDTH_3 = const(0b011)
BANDWIDTH_5 = const(0b101)
lpf_setting_values = (BANDWIDTH_1, BANDWIDTH_2, BANDWIDTH_3, BANDWIDTH_5)

# Acceleration Output Rate HZ

BANDWIDTH_50 = const(0x08)  # 50 Hz
BANDWIDTH_100 = const(0x09)  # 100 Hz
BANDWIDTH_125 = const(0xA)  # 125 Hz
BANDWIDTH_200 = const(0xB)  # 200 Hz
BANDWIDTH_250 = const(0xC)  # 250 Hz
BANDWIDTH_500 = const(0xD)  # 500 Hz
BANDWIDTH_1000 = const(0xE)  # 1000 Hz
BANDWIDTH_2000 = const(0xF)  # 2000 Hz
acceleration_output_data_rate_values = (
    BANDWIDTH_50,
    BANDWIDTH_100,
    BANDWIDTH_125,
    BANDWIDTH_200,
    BANDWIDTH_250,
    BANDWIDTH_500,
    BANDWIDTH_1000,
    BANDWIDTH_2000,
)


import struct

class IMU:
    """Driver for the MC3479 Sensor connected over I2C.

    :param ~machine.I2C i2c: The I2C bus the MC3479 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x4C`

    :raises RuntimeError: if the sensor is not found

    **Quickstart: Importing and using the device**

    Here is an example of using the :class:`micropython_mc3479.MC3479` class.
    First you will need to import the libraries to use the sensor

    .. code-block:: python

        from machine import Pin, I2C
        import micropython_mc3479 as MC3479

    Once this is done you can define your `machine.I2C` object and define your sensor object

    .. code-block:: python

        i2c = I2C(sda=Pin(8), scl=Pin(9))  # Correct I2C pins for UM FeatherS2
        mc3479 = MC3479.MC3479(i2c)

    Now you have access to the attributes

    .. code-block:: python

        accx, accy, accz = mc3479.acceleration

    """

    _device_id = RegisterStruct(_REG_WHOAMI, "B")
    _status = RegisterStruct(_SENSOR_STATUS_REG, "B")
    _mode_reg = RegisterStruct(_MODE_REG, "B")
    _range_scale_control = RegisterStruct(_ACC_RANGE, "B")
    _data_rate = RegisterStruct(_ACC_DATA_RATE, "B")

    # Acceleration Data
    _acc_data_x_msb = RegisterStruct(ACC_X_MSB, "B")
    _acc_data_x_lsb = RegisterStruct(ACC_X_LSB, "B")
    _acc_data_y_msb = RegisterStruct(ACC_Y_MSB, "B")
    _acc_data_y_lsb = RegisterStruct(ACC_Y_LSB, "B")
    _acc_data_z_msb = RegisterStruct(ACC_Z_MSB, "B")
    _acc_data_z_lsb = RegisterStruct(ACC_Z_LSB, "B")
    
    _mode = CBits(2, _MODE_REG, 0)

    # Acceleration Range Conf (0x20)
    _acc_range = CBits(3, _ACC_RANGE, 4)
    _acc_lpf_en = CBits(1, _ACC_RANGE, 3)
    _acc_lpf_setting = CBits(3, _ACC_RANGE, 0)


    acceleration_scale = {
        "ACCEL_RANGE_2G": 16384,
        "ACCEL_RANGE_4G": 8192,
        "ACCEL_RANGE_8G": 4096,
        "ACCEL_RANGE_16G": 2048,
        "ACCEL_RANGE_12G": 2730,
    }

    def __init__(self, i2c, address: int = 0x4C) -> None:
        self._i2c = i2c
        self._address = address
        if self._device_id != 0xA4:
            raise RuntimeError("Failed to find the MC3479 sensor")
        self._mode = NORMAL
        self.xOFF = 0 # This values must be checked and changed for every IMU (OFFSET)
        self.yOFF = 0
        self.zOFF = 0.4
        self.previousx = 0
        self.previousy = 0
        self.previousz = 0
                                          
    @property
    def acceleration(self) -> Tuple[float, float, float]:
        factor = self.acceleration_scale[self.acceleration_range]

        # Combine MSB and LSB into a 16-bit integer
        raw_x = (self._acc_data_x_msb << 8) | self._acc_data_x_lsb
        raw_y = (self._acc_data_y_msb << 8) | self._acc_data_y_lsb
        raw_z = (self._acc_data_z_msb << 8) | self._acc_data_z_lsb

        # Convert to signed integers using 2's complement
        x = raw_x if raw_x < 0x8000 else raw_x - 0x10000
        y = raw_y if raw_y < 0x8000 else raw_y - 0x10000
        z = raw_z if raw_z < 0x8000 else raw_z - 0x10000

        # Scale the values
        x /= factor
        y /= factor
        z /= factor
        
        # Low pass filter them
        accx = 0.6 * (x + self.xOFF) + 0.4 * self.previousx
        accy = 0.6 * (y + self.yOFF) + 0.4 * self.previousy
        accz = 0.6 * (z + self.zOFF) + 0.4 * self.previousz
        self.previousx = accx
        self.previousy = accy
        self.previousz = accz

        return accx, accy, accz

    @property
    def sensor_mode(self) -> str:
        """
        Standby
        ********
        * Lowest power consumption
        * Internal clocking is halted
        * No motion detection, sampling, or calibration
        * The I2C/SPI bus can read and write to registers (resolution, range, thresholds and other
          settings can be changed)
        * Reset not allowed
        * Default state after a power-up


        Normal
        *******
        * Highest power consumption
        * Internal clocking is enabled
        * Continuous motion detection and sampling; automatic calibration is available
        * The I2C/SPI bus can only write to the mode register and read all other registers
        * Reset allowed


        +----------------------------------------+-------------------------+
        | Mode                                   | Value                   |
        +========================================+=========================+
        | :py:const:`MC3479.STANDBY`             | :py:const:`0`           |
        +----------------------------------------+-------------------------+
        | :py:const:`MC3479.NORMAL`              | :py:const:`1`           |
        +----------------------------------------+-------------------------+


        """
        values = ("STANDBY", "NORMAL")
        return values[self._mode]

    @sensor_mode.setter
    def sensor_mode(self, value: int) -> None:
        if value not in (STANDBY, NORMAL):
            raise ValueError("Invalid Sensor Mode")
        self._mode = value

    @property
    def acceleration_range(self) -> str:
        """
        The range and scale control register sets the resolution, range,
        and filtering options for the accelerometer. All values are in
        sign-extended 2's complement format. Values are reported in
        registers 0x0D - 0x12 (the hardware formats the output)

        +----------------------------------------+-------------------------+
        | Mode                                   | Value                   |
        +========================================+=========================+
        | :py:const:`MC3479.ACCEL_RANGE_2G`      | :py:const:`0b000`       |
        +----------------------------------------+-------------------------+
        | :py:const:`MC3479.ACCEL_RANGE_4G`      | :py:const:`0b001`       |
        +----------------------------------------+-------------------------+
        | :py:const:`MC3479.ACCEL_RANGE_8G`      | :py:const:`0b010`       |
        +----------------------------------------+-------------------------+
        | :py:const:`MC3479.ACCEL_RANGE_16G`     | :py:const:`0b011`       |
        +----------------------------------------+-------------------------+
        | :py:const:`MC3479.ACCEL_RANGE_12G`     | :py:const:`0b100`       |
        +----------------------------------------+-------------------------+

        Example
        ########

        .. code-block:: python

            i2c = I2C(sda=Pin(8), scl=Pin(9))  # Correct I2C pins for UM FeatherS2
            mc3479 = MC3479.MC3479(i2c)
            mc3479.acceleration_range = MC3479.ACCEL_RANGE_12G

        """
        values = (
            "ACCEL_RANGE_2G",
            "ACCEL_RANGE_4G",
            "ACCEL_RANGE_8G",
            "ACCEL_RANGE_16G",
            "ACCEL_RANGE_12G",
        )
        return values[self._acc_range]

    @acceleration_range.setter
    def acceleration_range(self, value: int) -> None:
        if value not in accel_range_values:
            raise ValueError("Invalid Acceleration Range")
        self._mode = STANDBY
        self._acc_range = value
        self._mode = NORMAL

    @property
    def lpf_enabled(self) -> str:
        """
        Low Power Filter Enabler

        +----------------------------------------+-------------------------+
        | Mode                                   | Value                   |
        +========================================+=========================+
        | :py:const:`MC3479.LPF_ENABLE`          | :py:const:`0b0`         |
        +----------------------------------------+-------------------------+
        | :py:const:`MC3479.LPF_DISABLE`         | :py:const:`0b1`         |
        +----------------------------------------+-------------------------+

        Example
        ---------------------

        .. code-block:: python

            i2c = I2C(sda=Pin(8), scl=Pin(9))  # Correct I2C pins for UM FeatherS2
            mc3479 = MC3479.MC3479(i2c)
            mc3479.lpf_enabled = MC3479.LPF_ENABLE

        """
        values = ("LPF_DISABLE", "LPF_ENABLE")
        return values[self._acc_lpf_en]

    @lpf_enabled.setter
    def lpf_enabled(self, value: int) -> None:
        if value not in (LPF_ENABLE, LPF_DISABLE):
            raise ValueError("Invalid Low Pass Filter Setting")
        self._mode = STANDBY
        self._acc_lpf_en = value
        self._mode = NORMAL

    @property
    def lpf_setting(self) -> str:
        """
        Selects the Bandwidth for the Low Power Filter. Depends on the selection
        of the ODR/IDR

        +--------------------------------+------------------------------------+
        | Mode                           | Value                              |
        +================================+====================================+
        | :py:const:`MC3479.BANDWIDTH_1` | :py:const:`0b001` Fc = IDR / 4.255 |
        +--------------------------------+------------------------------------+
        | :py:const:`MC3479.BANDWIDTH_2` | :py:const:`0b010` Fc = IDR / 6     |
        +--------------------------------+------------------------------------+
        | :py:const:`MC3479.BANDWIDTH_3` | :py:const:`0b011` Fc = IDR / 12    |
        +--------------------------------+------------------------------------+
        | :py:const:`MC3479.BANDWIDTH_5` | :py:const:`0b101` Fc = IDR / 16    |
        +--------------------------------+------------------------------------+

        Example
        ---------------------

        .. code-block:: python

            i2c = I2C(sda=Pin(8), scl=Pin(9))  # Correct I2C pins for UM FeatherS2
            mc3479 = MC3479.MC3479(i2c)

            mc3479.lpf_setting = MC3479.BANDWIDTH_5

        """
        values = {
            1: "BANDWIDTH_1",
            2: "BANDWIDTH_2",
            3: "BANDWIDTH_3",
            5: "BANDWIDTH_5",
        }
        return values[self._acc_lpf_setting]

    @lpf_setting.setter
    def lpf_setting(self, value: int) -> None:
        if value not in lpf_setting_values:
            raise ValueError("Invalid Low Pass Filter Setting")
        self._mode = STANDBY
        self._acc_lpf_setting = value
        self._mode = NORMAL

    @property
    def acceleration_output_data_rate(self) -> str:
        """
        Define the output data rate in Hz
        The output data rate is dependent of the power mode setting for the sensor

        +----------------------------------------+---------------------------------+
        | Mode                                   | Value                           |
        +========================================+=================================+
        | :py:const:`MC3479.BANDWIDTH_50`        | :py:const:`0x08` 50 Hz          |
        +----------------------------------------+---------------------------------+
        | :py:const:`MC3479.BANDWIDTH_100`       | :py:const:`0x09` 100 Hz         |
        +----------------------------------------+---------------------------------+
        | :py:const:`MC3479.BANDWIDTH_125`       | :py:const:`0xA` 125 Hz          |
        +----------------------------------------+---------------------------------+
        | :py:const:`MC3479.BANDWIDTH_200`       | :py:const:`0xB` 200 Hz          |
        +----------------------------------------+---------------------------------+
        | :py:const:`MC3479.BANDWIDTH_250`       | :py:const:`0xC` 250 Hz          |
        +----------------------------------------+---------------------------------+
        | :py:const:`MC3479.BANDWIDTH_500`       | :py:const:`0xD` 500 Hz          |
        +----------------------------------------+---------------------------------+
        | :py:const:`MC3479.BANDWIDTH_1000`      | :py:const:`0xE` 1000 Hz         |
        +----------------------------------------+---------------------------------+
        | :py:const:`MC3479.BANDWIDTH_2000`      | :py:const:`0xF` 2000 Hz         |
        +----------------------------------------+---------------------------------+

        Example
        ########

        .. code-block:: python

            i2c = I2C(sda=Pin(8), scl=Pin(9))  # Correct I2C pins for UM FeatherS2
            mc3479 = MC3479.MC3479(i2c)
            mc3479.acceleration_output_data_rate = MC3479.BANDWIDTH_500

        """
        values = {
            0x08: "BANDWIDTH_50",
            0x09: "BANDWIDTH_100",
            0xA: "BANDWIDTH_125",
            0xB: "BANDWIDTH_200",
            0xC: "BANDWIDTH_250",
            0xD: "BANDWIDTH_500",
            0xE: "BANDWIDTH_1000",
            0xF: "BANDWIDTH_2000",
        }
        return values[self._data_rate]

    @acceleration_output_data_rate.setter
    def acceleration_output_data_rate(self, value: int) -> None:
        if value not in acceleration_output_data_rate_values:
            raise ValueError("Invalid Output Data Rate")
        self._mode = STANDBY
        self._data_rate = value
        self._mode = NORMAL
                        
        
class StreamingMovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []
        self.sum = 0

    def process(self, value):
        self.values.append(value)
        self.sum += value
        if len(self.values) > self.window_size:
            self.sum -= self.values.pop(0)
        return float(self.sum) / len(self.values)        

class shakeDetection:
    def __init__(self, window_size = 1, threshold = 1.1):  #window_size determines for how long the shaking has to occur and threshold determines how strong it has to be
        self.runningAvg = StreamingMovingAverage(window_size)
        self.shake_threshold = threshold # This is the threshold for the continuous shaking motion (intensity&time)
        self.violent_threshold = 2 # This threshold detects violent motion on the bike independent of time
        self.timer = 0
        
    def detect(self, accx, accy, accz):
        self.accx = accx
        self.accy = accy
        self.accz = accz
        self.totalAcc = math.sqrt(self.accx**2 + self.accy**2 + self.accz**2)
        self.avgAcc = self.runningAvg.process(self.totalAcc)
        
        if (self.avgAcc >= self.shake_threshold) or self.totalAcc >= 1.3:
            return True
        else:
            return False
        
        
def calculate_rotation_matrix(roll, pitch, yaw):
    '''
    Used to transform the IMU reference frame into the bike frame 
    '''
    # Convert angles to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)

    # Rotation around X-axis (roll)
    R_x = [
        [1, 0, 0],
        [0, math.cos(roll_rad), -math.sin(roll_rad)],
        [0, math.sin(roll_rad), math.cos(roll_rad)]
    ]

    # Rotation around Y-axis (pitch)
    R_y = [
        [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
        [0, 1, 0],
        [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
    ]

    # Rotation around Z-axis (yaw)
    R_z = [
        [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
        [math.sin(yaw_rad), math.cos(yaw_rad), 0],
        [0, 0, 1]
    ]

    # Overall rotation matrix
    R = [[sum(a*b for a, b in zip(R_x_row, R_y_col)) for R_y_col in zip(*R_y)] for R_x_row in R_x]
    R = [[sum(a*b for a, b in zip(R_row, R_z_col)) for R_z_col in zip(*R_z)] for R_row in R]

    return R


class tiltDetection:
    '''
    Used for making sure that the bike is being kept upright and not laying on the ground
    '''
    def __init__(self, tilt_threshold = 70, time_threshold = 5): # Define for how long the bike must be tilted for Detection
        self.tilt_threshold = tilt_threshold
        self.time_threshold = time_threshold
        self.restingX = 0.2432  # These values correspond to the approximate axis measurements when the bike is resting upright
        self.restingY = 0.7799
        self.restingZ = 0.6380
        self.restingXAngle = 0 # The only change
        self.restingYAngle = 55
        self.restingZAngle = 0
        self.rotationMatrix = calculate_rotation_matrix(self.restingYAngle, self.restingXAngle, self.restingZAngle)
        self.tiltTimer = 0
        self.tilting = False
        self.tiltAlarm = False
        self.previousAngle = 0
        
    def calibrate(self, accx, accy, accz): # Can be called to calibrate the bike upright/resting condition default
        self.restingX = accx
        self.restingY = accy
        self.restingZ = accz
        self.restingXAngle = math.atan2(accy, accz) * 180 / math.pi
        self.restingYAngle = math.atan2(-accx, math.sqrt(accy**2 + accz**2)) * 180 / math.pi
        
    def detect(self, accx, accy, accz):
        self.accx = accx 
        self.accy = accy 
        self.accz = accz
        self.totalAcc = math.sqrt(self.accx**2 + self.accy**2 + self.accz**2)

        acc_vector = [[accx], [accy], [accz]]
        self.transformed_acc_vector = [[sum(a*b for a, b in zip(row, acc_vector_col)) for acc_vector_col in zip(*acc_vector)] for row in self.rotationMatrix]
        self.transformed_acc = [self.transformed_acc_vector[0][0], self.transformed_acc_vector[1][0], self.transformed_acc_vector[2][0]]
        self.accx = self.transformed_acc[0]
        self.accy = self.transformed_acc[1]
        self.accz = self.transformed_acc[2]
        self.angleX = math.atan2(self.accy, self.accz) * 180 / math.pi
        self.angleY = math.atan2(-self.accx, math.sqrt(self.accy**2 + self.accz**2)) * 180 / math.pi
        self.rollAngle = self.angleY * 1.8
        
        if math.fabs(math.fabs(self.totalAcc) - 1) <= 0.2: # Check if the bike is only being affected by gravity, meaning totalAcc should be pretty close to 1
            
            self.val = self.previousAngle * 0.95 + self.rollAngle * 0.05
            self.previousAngle = self.rollAngle
            if math.fabs(self.val) >= self.tilt_threshold:
                self.tiltAlarm = True
            else:
                self.tiltAlarm = False

        return self.rollAngle, self.tiltAlarm
        
        
        
'''
FUNCTIONAL EXAMPLE
_________________
'''
i2c = SoftI2C(sda=Pin(21), scl=Pin(22)) 
mc3479 = IMU(i2c)
shakeDetector = shakeDetection(window_size = 1, threshold = 1.1)
tiltDetector = tiltDetection()
while True:
    accx, accy, accz = mc3479.acceleration
    print(f"x:{accx:.2f}m/s^2  y:{accy:.2f}m/s^2  z:{accz:.2f}m/s^2")
    shaking = shakeDetector.detect(accx, accy, accz)
    tilting = tiltDetector.detect(accx, accy, accz)
    #if shaking:
        #print(shaking)
    time.sleep(0.1)