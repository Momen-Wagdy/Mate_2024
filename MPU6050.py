#!/usr/bin/env python

import rospy
from smbus import SMBus
from sensor_msgs.msg import Imu

# MPU6050 Registers
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B

# MPU6050 I2C Address
MPU6050_ADDR = 0x68

# Accelerometer Sensitivity Scale Factor
ACCEL_SCALE_FACTOR = 16384.0

# Gyroscope Sensitivity Scale Factor
GYRO_SCALE_FACTOR = 131.0

# Initialize ROS node
rospy.init_node('mpu6050_imu_publisher')

# Initialize ROS publisher
imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)

# Initialize I2C bus
bus = SMBus(1)  # Use /dev/i2c-1

# Setup MPU6050
bus.write_byte_data(MPU6050_ADDR, REG_PWR_MGMT_1, 0)

def read_imu_data():
    # Read accelerometer data
    accel_data = [bus.read_byte_data(MPU6050_ADDR, REG_ACCEL_XOUT_H + i) for i in range(6)]
    ax = (accel_data[0] << 8 | accel_data[1])
    ay = (accel_data[2] << 8 | accel_data[3])
    az = (accel_data[4] << 8 | accel_data[5])

    # Convert to acceleration in m/s^2
    ax = ax / ACCEL_SCALE_FACTOR
    ay = ay / ACCEL_SCALE_FACTOR
    az = az / ACCEL_SCALE_FACTOR

    # Create Imu message
    imu_msg = Imu()
    imu_msg.linear_acceleration.x = ax
    imu_msg.linear_acceleration.y = ay
    imu_msg.linear_acceleration.z = az

    return imu_msg

if __name__ == '__main__':
    try:
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            imu_msg = read_imu_data()
            imu_pub.publish(imu_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass



