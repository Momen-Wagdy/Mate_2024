import RPi.GPIO as GPIO  # Import the RPi.GPIO library for interacting with GPIO pins
import time             # Import the time library for timing-related functions

# Set GPIO mode to use physical pin numbering
GPIO.setmode(GPIO.BOARD)

# Define the GPIO pin connected to the thruster (MEE6)
THRUSTER_PIN = 11

# Define the PWM frequency
PWM_FREQ = 50  # Hz

# Define maximum and minimum throttle values for the thruster (MEE6)
MAX_THROTTLE = 5000
MIN_THROTTLE = 0

# Initialize throttle value
throttle = 0

# Setup thruster (MEE6) pin as an output
GPIO.setup(THRUSTER_PIN, GPIO.OUT)

# Create a PWM object for the thruster (MEE6) pin with the specified frequency
pwm = GPIO.PWM(THRUSTER_PIN, PWM_FREQ)

# Start PWM with an initial duty cycle of 0
pwm.start(0)

# Function to set throttle for the thruster (MEE6)
def set_throttle(val):
    global throttle  # Access the global throttle variable
    
    # Map the input throttle value (0-100) to the thruster (MEE6) range (0-5000)
    mapped_val = int((val / 100.0) * 5000)

    # Ensure mapped value is within acceptable range
    if mapped_val < MIN_THROTTLE:
        mapped_val = MIN_THROTTLE
    elif mapped_val > MAX_THROTTLE:
        mapped_val = MAX_THROTTLE

    # Update throttle value and set PWM duty cycle
    throttle = mapped_val
    pwm.ChangeDutyCycle(throttle)

try:
    while True:
        # Prompt user for throttle input (0-100) for the thruster (MEE6)
        throttle_input = int(input("Enter throttle value (0-100) for MEE6 thruster: "))

        # Set the throttle for the thruster (MEE6) based on user input
        set_throttle(throttle_input)

except KeyboardInterrupt:
    # Clean up GPIO if Ctrl+C is pressed
    pwm.stop()        # Stop PWM
    GPIO.cleanup()    # Reset GPIO pins to their default state
