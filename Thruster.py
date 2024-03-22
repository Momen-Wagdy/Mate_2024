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
    global throttle

    # Map the input throttle value (0-100) to the desired range (0-5000)
    mapped_val = int((val / 100.0) * 5000)

    # Check if the mapped value is within the acceptable range
    if mapped_val < MIN_THROTTLE:
        mapped_val = MIN_THROTTLE
    elif mapped_val > MAX_THROTTLE:
        mapped_val = MAX_THROTTLE

    throttle = mapped_val
    pwm.ChangeDutyCycle(throttle)

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
