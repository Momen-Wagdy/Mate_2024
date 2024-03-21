import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Set GPIO pins for ESC control
ESC_PIN = 11

# Set frequency for PWM
PWM_FREQ = 50  # Hz

# Set maximum and minimum throttle values
MAX_THROTTLE = 5000
MIN_THROTTLE = 0

# Initialize throttle value
throttle = 0

# Initialize GPIO pin for ESC
GPIO.setup(ESC_PIN, GPIO.OUT)

# Create PWM object
pwm = GPIO.PWM(ESC_PIN, PWM_FREQ)

# Start PWM with 0 throttle
pwm.start(0)

# Function to set throttle
def set_throttle(val):
    global throttle

    # Scale the throttle input from (0-100) to (MIN_THROTTLE-MAX_THROTTLE)
    throttle = MIN_THROTTLE + (val / 100.0) * (MAX_THROTTLE - MIN_THROTTLE)

    # Ensure throttle is within bounds
    if throttle < MIN_THROTTLE:
        throttle = MIN_THROTTLE
    elif throttle > MAX_THROTTLE:
        throttle = MAX_THROTTLE

    # Change the duty cycle of the PWM signal to control the motor speed
    pwm.ChangeDutyCycle(throttle / 100.0)

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
