import pygame
import time

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

def read_joystick(joystick):
    while True:
        pygame.event.pump()  # Pump the event queue to capture events
        # Read joystick axes
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            print("Axis", i, "value:", axis_value)
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
