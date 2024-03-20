#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

# Define the ROS node class
class ThrusterControlNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('thruster_control_node', anonymous=True)

        # Set the frequency at which to publish throttle commands (in Hz)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Set the maximum and minimum throttle values
        self.MAX_THROTTLE = 100
        self.MIN_THROTTLE = 0

        # Initialize throttle value
        self.throttle = 0

        # Create a publisher for throttle commands
        self.throttle_pub = rospy.Publisher('thruster_throttle', Int32, queue_size=10)

    # Function to set throttle
    def set_throttle(self, val):
        # Check if the provided throttle value is within bounds
        if val < self.MIN_THROTTLE:
            val = self.MIN_THROTTLE
        elif val > self.MAX_THROTTLE:
            val = self.MAX_THROTTLE

        self.throttle = val

    # Main loop
    def run(self):
        while not rospy.is_shutdown():
            # Get throttle input from user
            throttle_input = int(input("Enter throttle value (0-100): "))

            # Set throttle
            self.set_throttle(throttle_input)

            # Publish throttle command
            self.throttle_pub.publish(self.throttle)

            # Sleep to maintain desired loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Create an instance of the ThrusterControlNode class
        node = ThrusterControlNode()

        # Run the node
        node.run()
    except rospy.ROSInterruptException:
        pass
