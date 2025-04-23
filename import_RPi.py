import RPi.GPIO as GPIO
import time
import subprocess

BUTTON_PIN = 17  # GPIO pin where the button is connected

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
print("Waiting for button press...")

try:
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            print("Button pressed!")
            subprocess.run(["python3", "/path/to/my_script.py"])
            time.sleep(0.5)  # debounce delay
except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()
