import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Set GPIO pins for ESC control
ESC_PIN = 11

# Set frequency for PWM
PWM_FREQ = 50  # Hz

# Set maximum and minimum throttle values
MAX_THROTTLE = 100
MIN_THROTTLE = 0

# Set initial throttle value
throttle = 0

# Initialize GPIO pin for ESC
GPIO.setup(ESC_PIN, GPIO.OUT)

# Create PWM object
pwm = GPIO.PWM(ESC_PIN, PWM_FREQ)

# Start PWM with 0 throttle
pwm.start(0)

# Function to set throttle
def set_throttle(val):
    # Declare the function set_throttle which takes a single argument val
    global throttle
    # Declare throttle as a global variable so it can be accessed and modified within this function

    # Check if the provided throttle value is less than the minimum throttle value
    if val < MIN_THROTTLE:
        val = MIN_THROTTLE  # If it is, set val to the minimum throttle value
    # Check if the provided throttle value is greater than the maximum throttle value
    elif val > MAX_THROTTLE:
        val = MAX_THROTTLE  # If it is, set val to the maximum throttle value

    throttle = val  # Set the global variable throttle to the adjusted throttle value
    pwm.ChangeDutyCycle(throttle)  # Change the duty cycle of the PWM signal to control the motor speed

try:
    while True:
        # Get throttle input from user
        throttle_input = int(input("Enter throttle value (0-100): "))

        # Set throttle
        set_throttle(throttle_input)

except KeyboardInterrupt:
    # Clean up GPIO
    pwm.stop()
    GPIO.cleanup()
