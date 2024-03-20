
# pip install RPi.GPIO


import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BOARD)

# Set the GPIO pin for servo
servo_pin = 12

# Set the PWM pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Create PWM instance with frequency
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz (20 ms period)

# Start PWM with 0 duty cycle
pwm.start(0)

# Function to set angle of the servo
def set_angle(angle):
    # Calculate the duty cycle required for the given angle
    duty = angle / 18 + 2

    # Turn on the servo by setting the GPIO pin high
    GPIO.output(servo_pin, True)

    # Change the duty cycle of the PWM signal to move the servo to the desired angle
    pwm.ChangeDutyCycle(duty)

    # Wait for the servo to move to the desired position
    time.sleep(1)  # Adjust this sleep time as needed

    # Turn off the servo by setting the GPIO pin low
    GPIO.output(servo_pin, False)

    # Reset the duty cycle to 0 to stop sending PWM signal
    pwm.ChangeDutyCycle(0)


try:
    while True:
        # Move servo to 0 degrees
        set_angle(0)
        time.sleep(1)

        # Move servo to 90 degrees
        set_angle(90)
        time.sleep(1)

        # Move servo to 180 degrees
        set_angle(180)
        time.sleep(1)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
