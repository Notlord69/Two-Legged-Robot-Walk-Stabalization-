#!/usr/bin/env python3

import time

LED_STATE = False  # False = OFF, True = ON

def toggle_led(state):
    if state:
        print("LED ON")
    else:
        print("LED OFF")

if __name__ == "__main__":
    try:
        while True:
            LED_STATE = not LED_STATE
            toggle_led(LED_STATE)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nSimulation stopped.")
