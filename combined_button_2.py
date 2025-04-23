import paho.mqtt.client as mqtt
import re
import numpy as np
from scipy.optimize import least_squares
import RPi.GPIO as GPIO
import time
import math

# Globals
latest_position = None

# GPIO pin setup
GPIO.setmode(GPIO.BCM)
SERVO_X_PIN = 18
SERVO_Y_PIN = 27
LASER_PIN = 25
BUTTON_PIN = 22

GPIO.setup(SERVO_X_PIN, GPIO.OUT)
GPIO.setup(SERVO_Y_PIN, GPIO.OUT)
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

servo_x = GPIO.PWM(SERVO_X_PIN, 50)
servo_y = GPIO.PWM(SERVO_Y_PIN, 50)
servo_x.start(0)
servo_y.start(0)

# Distance history
distance_history = {
    "ULCORNER": [],
    "URCORNER": [],
    "BLCORNER": [],
    "BRCORNER": []
}

MAX_HISTORY_LENGTH = 10

# MQTT broker details
mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topics = [
    "esp32/distance/ULCORNER",
    "esp32/distance/URCORNER",
    "esp32/distance/BLCORNER",
    "esp32/distance/BRCORNER"
]

# Reference points
ref1 = np.array([-157.5, 0, -233.7])
ref2 = np.array([-657.86, 955.0, -137.16])
ref3 = np.array([274.3, 492.8, -149.9])
ref4 = np.array([274.3, 67.31, -149.9])

# Functions
def update_history(esp32_id, new_distance):
    if esp32_id in distance_history:
        distance_history[esp32_id].append(new_distance)
        if len(distance_history[esp32_id]) > MAX_HISTORY_LENGTH:
            distance_history[esp32_id].pop(0)

def extract_distance(data):
    match = re.search(r'(\d+)\s*cm', data)
    return float(match.group(1)) if match else None

def calculate_average(arr):
    if not arr:
        return 0
    arr_np = np.array(arr)
    mean = np.mean(arr_np)
    std = np.std(arr_np)
    filtered = arr_np[(arr_np >= mean - 0.8 * std) & (arr_np <= mean + 0.8 * std)]
    if len(filtered) == 0:
        filtered = arr_np
    alpha = 0.3
    smoothed = filtered[0]
    for val in filtered[1:]:
        smoothed = alpha * val + (1 - alpha) * smoothed
    return smoothed

def triangulate_position():
    global latest_position
    if all(len(distance_history[k]) >= 3 for k in ["ULCORNER", "URCORNER", "BLCORNER", "BRCORNER"]):
        d1 = calculate_average(distance_history["BLCORNER"])
        d2 = calculate_average(distance_history["ULCORNER"])
        d3 = calculate_average(distance_history["URCORNER"])
        d4 = calculate_average(distance_history["BRCORNER"])

        def my_system(vars):
            x, y, z = vars
            F = np.zeros(4)
            F[0] = np.sqrt((x - ref1[0])**2 + (y - ref1[1])**2 + (z - ref1[2])**2) - d1
            F[1] = np.sqrt((x - ref2[0])**2 + (y - ref2[1])**2 + (z - ref2[2])**2) - d2
            F[2] = np.sqrt((x - ref3[0])**2 + (y - ref3[1])**2 + (z - ref3[2])**2) - d3
            F[3] = np.sqrt((x - ref4[0])**2 + (y - ref4[1])**2 + (z - ref4[2])**2) - d4
            return F

        # Solve for the unknown point
        initial_guess = [0.5, 0.5, 0.5]  # Starting guess for (x, y, z)
        result = least_squares(my_system, initial_guess)
        
        latest_position = np.round(result.x, decimals=4)
        
        print("Unknown point:", latest_position)

def on_message(client, userdata, msg):
    esp32_id = msg.topic.split("/")[-1]
    data = msg.payload.decode().strip()
    distance = extract_distance(data)
    if distance is not None:
        update_history(esp32_id, distance)
        triangulate_position()

def cartesian_to_servo_angles(x, y, z):
    horizontal_angle = -(math.degrees(math.atan2(x, y)) - 180) - 180
    p = math.sqrt(x**2 + y**2)
    vertical_angle = -90 - math.degrees(math.atan(z / p))
    return horizontal_angle, vertical_angle

def angle_to_duty_cycle(angle):
    return max(2, min(12, 2 + (angle + 90) * 10 / 180))

def turn_laser_on():
    GPIO.output(LASER_PIN, GPIO.HIGH)

def turn_laser_off():
    GPIO.output(LASER_PIN, GPIO.LOW)

def point_laser_at_position(position):
    target_x, target_y, target_z = position
    target_horizontal_angle, target_vertical_angle = cartesian_to_servo_angles(target_x, target_y, target_z)
    target_horizontal_angle -= 20
    target_vertical_angle += 7

    current_horizontal_angle = 0
    current_vertical_angle = 0
    step_size = 1

    turn_laser_on()
    time.sleep(0.5)

    # Horizontal movement
    while round(current_horizontal_angle, 1) != round(target_horizontal_angle, 1):
        if current_horizontal_angle < target_horizontal_angle:
            current_horizontal_angle = min(current_horizontal_angle + step_size, target_horizontal_angle)
        else:
            current_horizontal_angle = max(current_horizontal_angle - step_size, target_horizontal_angle)

        duty = angle_to_duty_cycle(current_horizontal_angle)
        servo_x.ChangeDutyCycle(duty)
        time.sleep(0.1)

    servo_x.ChangeDutyCycle(0)  # Stop signal
    time.sleep(0.2)

    # Vertical movement
    while round(current_vertical_angle, 1) != round(target_vertical_angle, 1):
        if current_vertical_angle < target_vertical_angle:
            current_vertical_angle = min(current_vertical_angle + step_size, target_vertical_angle)
        else:
            current_vertical_angle = max(current_vertical_angle - step_size, target_vertical_angle)

        duty = angle_to_duty_cycle(current_vertical_angle)
        servo_y.ChangeDutyCycle(duty)
        time.sleep(0.1)

    servo_y.ChangeDutyCycle(0)  # Stop signal
    time.sleep(0.2)

    # Keep laser on briefly then turn off
    time.sleep(10)
    turn_laser_off()


# MQTT setup
client = mqtt.Client()
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)
for topic in mqtt_topics:
    client.subscribe(topic)

print("Subscribed to MQTT topics.")
print("Listening for button press...")

try:
    while True:
        client.loop(timeout=0.1)

        if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            print("Button pressed!")
            if latest_position is not None:
                point_laser_at_position(latest_position)
            else:
                print("Not enough data to triangulate yet.")
            time.sleep(0.5)  # Debounce delay

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    servo_x.stop()
    servo_y.stop()
    turn_laser_off()
    GPIO.cleanup()
    print("GPIO cleaned up.")
