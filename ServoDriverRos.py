#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time

class ServoControlNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('servo_control_node', anonymous=True)

        # Set the GPIO numbering mode
        GPIO.setmode(GPIO.BCM)

        # Set the GPIO pin to which your servo is connected
        self.servo_pin = 18

        # Set the PWM frequency (Hz)
        self.PWM_freq = 50

        # Setup the GPIO pin as an output
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # Create a PWM instance
        self.pwm = GPIO.PWM(self.servo_pin, self.PWM_freq)

        # Initialize ROS publisher
        self.servo_angle_pub = rospy.Publisher('/servo_angle', Float64, queue_size=10)

        # Set the loop rate for publishing servo angle
        self.rate = rospy.Rate(10)  # 10 Hz

    # Function to convert angle to duty cycle
    def angle_to_duty_cycle(self, angle):
        duty_cycle = (angle / 18) + 2
        return duty_cycle

    # Function to set the servo angle
    def set_angle(self, angle):
        # Convert the desired angle to duty cycle using the angle_to_duty_cycle function
        duty_cycle = self.angle_to_duty_cycle(angle)
        
        # Start the PWM signal with the calculated duty cycle
        self.pwm.start(duty_cycle)
        
        # Publish the servo angle
        self.servo_angle_pub.publish(angle)

        # Sleep for a short duration to allow the servo to move
        time.sleep(1)

        # Stop the PWM signal after the delay to prevent the servo from continuously moving
        self.pwm.stop()

    # Function to run the ROS node
    def run(self):
        try:
            while not rospy.is_shutdown():
                # Example usage
                self.set_angle(0)   # Change the angle value (0 to 180) to move the servo
                self.rate.sleep()
                self.set_angle(90)  # Change the angle value (0 to 180) to move the servo
                self.rate.sleep()
                self.set_angle(180) # Change the angle value (0 to 180) to move the servo
                self.rate.sleep()

        except KeyboardInterrupt:
            self.pwm.stop()
            GPIO.cleanup()

if __name__ == '__main__':
    try:
        node = ServoControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
