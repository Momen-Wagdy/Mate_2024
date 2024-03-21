import RPi.GPIO as GPIO
import time

# Set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin connected to the LED
LED_PIN = 18

# Set the GPIO pin as an output
GPIO.setup(LED_PIN, GPIO.OUT)

try:
    while True:
        # Turn on the LED
        GPIO.output(LED_PIN, GPIO.HIGH)
        print("LED is ON")
        time.sleep(1)  # Wait for 1 second

        # Turn off the LED
        GPIO.output(LED_PIN, GPIO.LOW)
        print("LED is OFF")
        time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
    # Clean up GPIO on exit
    GPIO.cleanup()
