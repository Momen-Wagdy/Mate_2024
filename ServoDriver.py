import RPi.GPIO as GPIO
import time

# Set the GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set the GPIO pin to which your servo is connected
servo_pin = 18

# Set the PWM frequency (Hz)
PWM_freq = 50

# Setup the GPIO pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Create a PWM instance
pwm = GPIO.PWM(servo_pin, PWM_freq)

# Function to convert angle to duty cycle
def angle_to_duty_cycle(angle):
    # Duty cycle formula: (angle / 18) + 2
    # This formula converts the desired angle (in degrees) to the corresponding duty cycle.
    # Servo motors typically have a range of motion of 0 to 180 degrees,
    # which corresponds to a duty cycle range of approximately 2 to 12.
    duty_cycle = (angle / 18) + 2
    return duty_cycle

# Function to set the servo angle
def set_angle(angle):
    # Convert the desired angle to duty cycle using the angle_to_duty_cycle function
    duty_cycle = angle_to_duty_cycle(angle)
    
    # Start the PWM signal with the calculated duty cycle
    pwm.start(duty_cycle)
    
    # Time delay to keep the servo in position
    # Adjust this value to control how long the servo stays in the desired position
    time.sleep(1)
    
    # Stop the PWM signal after the delay to prevent the servo from continuously moving
    pwm.stop()


# Example usage
try:
    while True:
        set_angle(0)  # Change the angle value (0 to 180) to move the servo
        time.sleep(1)  # Adjust the delay as needed
        set_angle(90) # Change the angle value (0 to 180) to move the servo
        time.sleep(1)  # Adjust the delay as needed
        set_angle(180) # Change the angle value (0 to 180) to move the servo
        time.sleep(1)  # Adjust the delay as needed

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
