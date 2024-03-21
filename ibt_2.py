import RPi.GPIO as GPIO
import time

# Define GPIO pins for motor control
right_r_en = 3
right_l_en = 5
left_r_en = 6
left_l_en = 9

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(right_r_en, GPIO.OUT)
GPIO.setup(right_l_en, GPIO.OUT)
GPIO.setup(left_r_en, GPIO.OUT)
GPIO.setup(left_l_en, GPIO.OUT)

def setup():
    GPIO.setup(right_r_en, GPIO.OUT)
    GPIO.setup(right_l_en, GPIO.OUT)
    GPIO.setup(left_r_en, GPIO.OUT)
    GPIO.setup(left_l_en, GPIO.OUT)

def up():
    GPIO.output(right_r_en, GPIO.HIGH)
    GPIO.output(right_l_en, GPIO.LOW)
    GPIO.output(left_r_en, GPIO.HIGH)
    GPIO.output(left_l_en, GPIO.LOW)

def down():
    GPIO.output(right_r_en, GPIO.LOW)
    GPIO.output(right_l_en, GPIO.HIGH)
    GPIO.output(left_r_en, GPIO.LOW)
    GPIO.output(left_l_en, GPIO.HIGH)

def loop():
    # put your main code here, to run repeatedly
    pass

def cleanup():
    GPIO.cleanup()

try:
    setup()
    up()
    time.sleep(5)  # Move up for 5 seconds (adjust as needed)
    down()
    time.sleep(5)  # Move down for 5 seconds (adjust as needed)

except KeyboardInterrupt:
    pass

finally:
    cleanup()
