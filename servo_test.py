import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
SERVO_PIN = 23
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN,50)
pwm.start(0)

try:
	while True:
		print("0 degrees")
		pwm.ChangeDutyCycle(2)
		time.sleep(2)
		print("90 degrees")
		pwm.ChangeDutyCycle(7)
		time.sleep(2)
		print("180 degrees")
		pwm.ChangeDutyCycle(12)
		time.sleep(2)
except KeyboardInterrupt:
	print("Stopping")
finally:
	pwm.stop()
	GPIO.cleanup()
