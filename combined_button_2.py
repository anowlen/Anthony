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
            return np.array([
                np.linalg.norm([x - ref1[0], y - ref1[1], z - ref1[2]]) - d1,
                np.linalg.norm([x - ref2[0], y - ref2[1], z - ref2[2]]) - d2,
                np.linalg.norm([x - ref3[0], y - ref3[1], z - ref3[2]]) - d3,
                np.linalg.norm([x - ref4[0], y - ref4[1], z - ref4[2]]) - d4
            ])

        result = least_squares(my_system, [100, 100, 50])
        latest_position = np.round(result.x, decimals=4)
        print("Triangulated Position:", latest_position)

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
    h_angle, v_angle = cartesian_to_servo_angles(target_x, target_y, target_z)
    h_angle -= 20
    v_angle += 7

    cur_h = 0
    cur_v = 0
    step = 1

    turn_laser_on()
    time.sleep(1)

    while round(cur_h, 1) != round(h_angle, 1):
        cur_h = cur_h + step if cur_h < h_angle else cur_h - step
        servo_x.ChangeDutyCycle(angle_to_duty_cycle(cur_h))
        time.sleep(0.1)
        servo_x.ChangeDutyCycle(0)

    while round(cur_v, 1) != round(v_angle, 1):
        cur_v = cur_v + step if cur_v < v_angle else cur_v - step
        servo_y.ChangeDutyCycle(angle_to_duty_cycle(cur_v))
        time.sleep(0.1)
        servo_y.ChangeDutyCycle(0)

    time.sleep(5)
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
