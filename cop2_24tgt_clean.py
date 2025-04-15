import RPi.GPIO as GPIO
import time
import math

# Setup
GPIO.setmode(GPIO.BCM)
SERVO_X_PIN = 18
SERVO_Y_PIN = 17
LASER_PIN = 25
GPIO.setup(SERVO_X_PIN, GPIO.OUT)
GPIO.setup(SERVO_Y_PIN, GPIO.OUT)
GPIO.setup(LASER_PIN, GPIO.OUT)
servo_x = GPIO.PWM(SERVO_X_PIN, 50)
servo_y = GPIO.PWM(SERVO_Y_PIN, 50)
servo_x.start(0)
servo_y.start(0)

# Functions
def cartesian_to_servo_angles(x, y, z):
    horizontal_angle = -(math.degrees(math.atan2(x, y)) - 180) - 180
    p = math.sqrt(x**2 + y**2)
    vertical_angle = -90 - math.degrees(math.atan(z / p))
    return horizontal_angle, vertical_angle

def angle_to_duty_cycle(angle):
    return max(2, min(12, 2 + (angle + 90) * 10 / 180))

def turn_laser_on():
    GPIO.output(LASER_PIN, GPIO.HIGH)
    print("Laser ON")

# Target
target_x, target_y, target_z = 83, 340.4, -233.7
target_horizontal_angle, target_vertical_angle = cartesian_to_servo_angles(target_x, target_y, target_z)
target_horizontal_angle -= 20
target_vertical_angle += 7

# Initial
current_horizontal_angle = 0
current_vertical_angle = 0
step_size = 1

try:
    turn_laser_on()
    time.sleep(1)

    # Horizontal movement
    while round(current_horizontal_angle, 1) != round(target_horizontal_angle, 1):
        if current_horizontal_angle < target_horizontal_angle:
            current_horizontal_angle = min(current_horizontal_angle + step_size, target_horizontal_angle)
        else:
            current_horizontal_angle = max(current_horizontal_angle - step_size, target_horizontal_angle)

        duty = angle_to_duty_cycle(current_horizontal_angle)
        servo_x.ChangeDutyCycle(duty)
        time.sleep(0.1)
        servo_x.ChangeDutyCycle(0)  # prevent jitter

    # Vertical movement
    while round(current_vertical_angle, 1) != round(target_vertical_angle, 1):
        if current_vertical_angle < target_vertical_angle:
            current_vertical_angle = min(current_vertical_angle + step_size, target_vertical_angle)
        else:
            current_vertical_angle = max(current_vertical_angle - step_size, target_vertical_angle)

        duty = angle_to_duty_cycle(current_vertical_angle)
        servo_y.ChangeDutyCycle(duty)
        time.sleep(0.1)
        servo_y.ChangeDutyCycle(0)

    # Keep laser on
    time.sleep(300)

finally:
    servo_x.stop()
    servo_y.stop()
    GPIO.output(LASER_PIN, GPIO.LOW)
    GPIO.cleanup()
    print("GPIO cleaned up. Laser OFF.")

