import pygame
import time
import RPi.GPIO as GPIO

# Define GPIO pins for LEDs
LED_RED = 17
LED_GREEN = 18
LED_BLUE = 27

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_RED, GPIO.OUT)
GPIO.setup(LED_GREEN, GPIO.OUT)
GPIO.setup(LED_BLUE, GPIO.OUT)

def init_joystick():
    pygame.init()
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick detected")
        return None
    else:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print("Initialized joystick:", joystick.get_name())
        return joystick

def control_leds(axis_value):
    if axis_value < -0.5:
        GPIO.output(LED_RED, GPIO.HIGH)
        GPIO.output(LED_GREEN, GPIO.LOW)
        GPIO.output(LED_BLUE, GPIO.LOW)
    elif axis_value > 0.5:
        GPIO.output(LED_RED, GPIO.LOW)
        GPIO.output(LED_GREEN, GPIO.LOW)
        GPIO.output(LED_BLUE, GPIO.HIGH)
    else:
        GPIO.output(LED_RED, GPIO.LOW)
        GPIO.output(LED_GREEN, GPIO.HIGH)
        GPIO.output(LED_BLUE, GPIO.LOW)

def read_joystick(joystick):
    while True:
        pygame.event.pump()  # Pump the event queue to capture events
        # Read joystick axes
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            print("Axis", i, "value:", axis_value)
            control_leds(axis_value)
        # Read joystick buttons
        for i in range(joystick.get_numbuttons()):
            button_state = joystick.get_button(i)
            print("Button", i, "state:", button_state)
        time.sleep(0.1)  # Sleep to avoid flooding the console

def main():
    joystick = init_joystick()
    if joystick:
        read_joystick(joystick)
    else:
        print("Exiting.")

if __name__ == "__main__":
    main()
