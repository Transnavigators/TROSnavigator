# From https://github.com/mwilliams03/BerryIMU
import smbus
from LSM9DS0 import *
from LSM9DS1 import *
import time


class BerryIMU:
    def __init__(self, is_virtual=False, gyr_addr=0, mag_addr=0, acc_addr=0):
        self.LSM9DS0 = 0

        if is_virtual:
            self.bus = smbus.SMBus(0)
        else:
            self.bus = smbus.SMBus(1)
            # self.detectIMU()
        if self.LSM9DS0:
            self.MAG_ADDRESS = LSM9DS0_MAG_ADDRESS
            self.GYR_ADDRESS = LSM9DS0_GYR_ADDRESS
            self.ACC_ADDRESS = LSM9DS0_ACC_ADDRESS
            self.OUT_X_L_G = LSM9DS0_OUT_X_L_G
            self.OUT_X_H_G = LSM9DS0_OUT_X_H_G
            self.OUT_Y_L_G = LSM9DS0_OUT_Y_L_G
            self.OUT_Y_H_G = LSM9DS0_OUT_Y_H_G
            self.OUT_Z_L_G = LSM9DS0_OUT_Z_L_G
            self.OUT_Z_H_G = LSM9DS0_OUT_Z_H_G
            self.OUT_X_L_XL = LSM9DS0_OUT_X_L_A
            self.OUT_X_H_XL = LSM9DS0_OUT_X_H_A
            self.OUT_Y_L_XL = LSM9DS0_OUT_Y_L_A
            self.OUT_Y_H_XL = LSM9DS0_OUT_Y_H_A
            self.OUT_Z_L_XL = LSM9DS0_OUT_Z_L_A
            self.OUT_Z_H_XL = LSM9DS0_OUT_Z_H_A
            self.OUT_X_L_M = LSM9DS0_OUT_X_L_M
            self.OUT_X_H_M = LSM9DS0_OUT_X_H_M
            self.OUT_Y_L_M = LSM9DS0_OUT_Y_L_M
            self.OUT_Y_H_M = LSM9DS0_OUT_Y_H_M
            self.OUT_Z_L_M = LSM9DS0_OUT_Z_L_M
            self.OUT_Z_H_M = LSM9DS0_OUT_Z_H_M
        else:
            self.MAG_ADDRESS = LSM9DS1_MAG_ADDRESS
            self.GYR_ADDRESS = LSM9DS1_GYR_ADDRESS
            self.ACC_ADDRESS = LSM9DS1_ACC_ADDRESS
            self.OUT_X_L_G = LSM9DS1_OUT_X_L_G
            self.OUT_X_H_G = LSM9DS1_OUT_X_H_G
            self.OUT_Y_L_G = LSM9DS1_OUT_Y_L_G
            self.OUT_Y_H_G = LSM9DS1_OUT_Y_H_G
            self.OUT_Z_L_G = LSM9DS1_OUT_Z_L_G
            self.OUT_Z_H_G = LSM9DS1_OUT_Z_H_G
            self.OUT_X_L_XL = LSM9DS1_OUT_X_L_XL
            self.OUT_X_H_XL = LSM9DS1_OUT_X_H_XL
            self.OUT_Y_L_XL = LSM9DS1_OUT_Y_L_XL
            self.OUT_Y_H_XL = LSM9DS1_OUT_Y_H_XL
            self.OUT_Z_L_XL = LSM9DS1_OUT_Z_L_XL
            self.OUT_Z_H_XL = LSM9DS1_OUT_Z_H_XL
            self.OUT_X_L_M = LSM9DS1_OUT_X_L_M
            self.OUT_X_H_M = LSM9DS1_OUT_X_H_M
            self.OUT_Y_L_M = LSM9DS1_OUT_Y_L_M
            self.OUT_Y_H_M = LSM9DS1_OUT_Y_H_M
            self.OUT_Z_L_M = LSM9DS1_OUT_Z_L_M
            self.OUT_Z_H_M = LSM9DS1_OUT_Z_H_M
        if is_virtual:
            self.MAG_ADDRESS = mag_addr
            self.GYR_ADDRESS = gyr_addr
            self.ACC_ADDRESS = acc_addr
        self.initIMU()

    def detectIMU(self):
        # Detect which version of BerryIMU is connected.
        # BerryIMUv1 uses the LSM9DS0
        # BerryIMUv2 uses the LSM9DS1

        try:
            # Check for LSM9DS0
            # If no LSM9DS0 is connected, there will be an I2C bus error and the program will exit.
            # This section of code stops this from happening.
            LSM9DS0_WHO_G_response = (self.bus.read_byte_data(LSM9DS0_GYR_ADDRESS, LSM9DS0_WHO_AM_I_G))
            LSM9DS0_WHO_XM_response = (self.bus.read_byte_data(LSM9DS0_ACC_ADDRESS, LSM9DS0_WHO_AM_I_XM))
        except IOError:
            pass
        else:
            if (LSM9DS0_WHO_G_response == 0xd4) and (LSM9DS0_WHO_XM_response == 0x49):
                print("Found LSM9DS0")
                self.LSM9DS0 = 1

        try:
            # Check for LSM9DS1
            # If no LSM9DS1 is connected, there will be an I2C bus error and the program will exit.
            # This section of code stops this from happening.
            LSM9DS1_WHO_XG_response = (self.bus.read_byte_data(LSM9DS1_GYR_ADDRESS, LSM9DS1_WHO_AM_I_XG))
            LSM9DS1_WHO_M_response = (self.bus.read_byte_data(LSM9DS1_MAG_ADDRESS, LSM9DS1_WHO_AM_I_M))

        except IOError:
            pass
        else:
            if (LSM9DS1_WHO_XG_response == 0x68) and (LSM9DS1_WHO_M_response == 0x3d):
                print("Found LSM9DS1")
                self.LSM9DS0 = 0

        time.sleep(1)
        return self.LSM9DS0

    def writeAG(self, register, value):
        self.bus.write_byte_data(self.ACC_ADDRESS, register, value)
        return -1

    def writeACC(self, register, value):
        self.bus.write_byte_data(self.ACC_ADDRESS, register, value)
        return -1

    def writeMAG(self, register, value):
        self.bus.write_byte_data(self.MAG_ADDRESS, register, value)
        return -1

    def writeGRY(self, register, value):
        self.bus.write_byte_data(self.GYR_ADDRESS, register, value)
        return -1

    def readACCx(self):
        acc_l = self.bus.read_byte_data(self.ACC_ADDRESS, self.OUT_X_L_XL)
        acc_h = self.bus.read_byte_data(self.ACC_ADDRESS, self.OUT_X_H_XL)
        acc_combined = (acc_l | acc_h << 8)

        return acc_combined if acc_combined < 32768 else acc_combined - 65536

    def readACCy(self):
        acc_l = self.bus.read_byte_data(self.ACC_ADDRESS, self.OUT_Y_L_XL)
        acc_h = self.bus.read_byte_data(self.ACC_ADDRESS, self.OUT_Y_H_XL)
        acc_combined = (acc_l | acc_h << 8)

        return acc_combined if acc_combined < 32768 else acc_combined - 65536

    def readACCz(self):
        acc_l = self.bus.read_byte_data(self.ACC_ADDRESS, self.OUT_Z_L_XL)
        acc_h = self.bus.read_byte_data(self.ACC_ADDRESS, self.OUT_Z_H_XL)
        acc_combined = (acc_l | acc_h << 8)

        return acc_combined if acc_combined < 32768 else acc_combined - 65536

    def readMAGx(self):
        mag_l = self.bus.read_byte_data(self.MAG_ADDRESS, self.OUT_X_L_M)
        mag_h = self.bus.read_byte_data(self.MAG_ADDRESS, self.OUT_X_H_M)
        mag_combined = (mag_l | mag_h << 8)
        return mag_combined if mag_combined < 32768 else mag_combined - 65536

    def readMAGy(self):
        mag_l = self.bus.read_byte_data(self.MAG_ADDRESS, self.OUT_Y_L_M)
        mag_h = self.bus.read_byte_data(self.MAG_ADDRESS, self.OUT_Y_H_M)
        mag_combined = (mag_l | mag_h << 8)

        return mag_combined if mag_combined < 32768 else mag_combined - 65536

    def readMAGz(self):
        mag_l = self.bus.read_byte_data(self.MAG_ADDRESS, self.OUT_Z_L_M)
        mag_h = self.bus.read_byte_data(self.MAG_ADDRESS, self.OUT_Z_H_M)
        mag_combined = (mag_l | mag_h << 8)

        return mag_combined if mag_combined < 32768 else mag_combined - 65536

    def readGYRx(self):
        gyr_l = self.bus.read_byte_data(self.GYR_ADDRESS, self.OUT_X_L_G)
        gyr_h = self.bus.read_byte_data(self.GYR_ADDRESS, self.OUT_X_H_G)
        gyr_combined = (gyr_l | gyr_h << 8)

        return gyr_combined if gyr_combined < 32768 else gyr_combined - 65536

    def readGYRy(self):
        gyr_l = self.bus.read_byte_data(self.GYR_ADDRESS, self.OUT_Y_L_G)
        gyr_h = self.bus.read_byte_data(self.GYR_ADDRESS, self.OUT_Y_H_G)
        gyr_combined = (gyr_l | gyr_h << 8)

        return gyr_combined if gyr_combined < 32768 else gyr_combined - 65536

    def readGYRz(self):
        gyr_l = self.bus.read_byte_data(self.GYR_ADDRESS, self.OUT_Z_L_G)
        gyr_h = self.bus.read_byte_data(self.GYR_ADDRESS, self.OUT_Z_H_G)
        gyr_combined = (gyr_l | gyr_h << 8)

        return gyr_combined if gyr_combined < 32768 else gyr_combined - 65536

    def initIMU(self):
        if self.LSM9DS0:  # For BerryIMUv1
            # initialise the accelerometer
            self.writeACC(LSM9DS0_CTRL_REG1_XM, 0b01100111)  # z,y,x axis enabled, continuos update,  100Hz data rate
            self.writeACC(LSM9DS0_CTRL_REG2_XM, 0b00100000)  # +/- 16G full scale

            # initialise the magnetometer
            self.writeMAG(LSM9DS0_CTRL_REG5_XM, 0b11110000)  # Temp enable, M data rate = 50Hz
            self.writeMAG(LSM9DS0_CTRL_REG6_XM, 0b01100000)  # +/-12gauss
            self.writeMAG(LSM9DS0_CTRL_REG7_XM, 0b00000000)  # Continuous-conversion mode

            # initialise the gyroscope
            self.writeGRY(LSM9DS0_CTRL_REG1_G, 0b00001111)  # Normal power mode, all axes enabled
            self.writeGRY(LSM9DS0_CTRL_REG4_G, 0b00110000)  # Continuos update, 2000 dps full scale

        else:  # For BerryIMUv2
            # initialise the gyroscope
            self.writeGRY(LSM9DS1_CTRL_REG4, 0b00111000)  # z, y, x axis enabled for gyro
            self.writeGRY(LSM9DS1_CTRL_REG1_G, 0b10111000)  # Gyro ODR = 476Hz, 2000 dps
            self.writeGRY(LSM9DS1_ORIENT_CFG_G, 0b10111000)  # Swap orientation

            # initialise the accelerometer
            self.writeACC(LSM9DS1_CTRL_REG5_XL, 0b00111000)  # z, y, x axis enabled for accelerometer
            self.writeACC(LSM9DS1_CTRL_REG6_XL, 0b00101000)  # +/- 16g

            # initialise the magnetometer
            self.writeMAG(LSM9DS1_CTRL_REG1_M, 0b10011100)  # Temp compensation enabled,Low power mode mode,80Hz ODR
            self.writeMAG(LSM9DS1_CTRL_REG2_M, 0b01000000)  # +/-12gauss
            self.writeMAG(LSM9DS1_CTRL_REG3_M, 0b00000000)  # continuos update
            self.writeMAG(LSM9DS1_CTRL_REG4_M, 0b00000000)  # lower power mode for Z axis
