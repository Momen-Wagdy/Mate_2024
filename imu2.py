# Make sure you have enabled I2C on your Raspberry Pi
# /*Using Raspberry Pi Configuration Tool (raspi-config):
# 1-Open the terminal on your Raspberry Pi.
# 2-Run the following command to open the Raspberry Pi Configuration Tool:
# sudo raspi-config

# 3-Navigate to "Interfacing Options" using the arrow keys and hit Enter.
# 4-Scroll down to "I2C" and hit Enter.
# 5-Select "Yes" when prompted to enable I2C.
# 6-Reboot your Raspberry Pi for the changes to take effect:
# sudo reboot

#  and installed the smbus library (sudo apt-get install python3-smbus).
import smbus
import time

# MPU-6050 Register Addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

# MPU-6050 I2C address
MPU6050_ADDR = 0x68

# Initialize I2C bus
bus = smbus.SMBus(1)

def MPU_Init():
    # Wake up MPU-6050
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    # Set the sample rate to 1kHz
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    # Configure the accelerometer range (+/- 2g)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
    # Configure the gyroscope range (+/- 250 degrees/s)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr+1)
    value = ((high << 8) | low)
    if value > 32768:
        value -= 65536
    return value

def main():
    MPU_Init()
    while True:
        # Read accelerometer data
        accel_x = read_raw_data(ACCEL_XOUT_H)
        accel_y = read_raw_data(ACCEL_YOUT_H)
        accel_z = read_raw_data(ACCEL_ZOUT_H)
        # Read gyroscope data
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = accel_x/16384.0
        Ay = accel_y/16384.0
        Az = accel_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        print("Accelerometer: Ax=%.2f g, Ay=%.2f g, Az=%.2f g" % (Ax, Ay, Az))
        print("Gyroscope: Gx=%.2f deg/s, Gy=%.2f deg/s, Gz=%.2f deg/s" % (Gx, Gy, Gz))
        print("---------------------------------------")
        time.sleep(0.5)

if __name__ == "__main__":
    main()
