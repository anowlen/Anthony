import paho.mqtt.client as mqtt
import re
import numpy as np
from scipy.optimize import least_squares
import RPi.GPIO as GPIO
import time
import math

# Global for latest triangulated position
latest_position = None

# Setup GPIO
GPIO.setmode(GPIO.BCM)
SERVO_X_PIN = 18
SERVO_Y_PIN = 17
LASER_PIN = 25
BUTTON_PIN = 23

GPIO.setup(SERVO_X_PIN, GPIO.OUT)
GPIO.setup(SERVO_Y_PIN, GPIO.OUT)
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Active-low button

servo_x = GPIO.PWM(SERVO_X_PIN, 50)
servo_y = GPIO.PWM(SERVO_Y_PIN, 50)
servo_x.start(0)
servo_y.start(0)

# Dictionary to store recent distances
distance_history = {
    "ULCORNER": [],
    "URCORNER": [],
    "BLCORNER": [],
    "BRCORNER": [],
    "RECENT": []
}

MAX_HISTORY_LENGTH = 10

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

# Reference points (corners)
ref1 = np.array([-157.5, 0, -233.7])       # bottom-left
ref2 = np.array([-657.86, 955.0, -137.16]) # top-left
ref3 = np.array([274.3, 492.8, -149.9])    # top-right
ref4 = np.array([274.3, 67.31, -149.9])    # bottom-right

# MQTT message callback
def on_message(client, userdata, msg):
    global latest_position

    esp32_id = msg.topic.split("/")[-1]
    data = msg.payload.decode().strip()
    distance = extract_distance(data)

    if distance is not None:
        update_history(esp32_id, distance)

        print("\nUpdated Distance History:")
        for key, values in distance_history.items():
            print(f"{key}: {values}")

        if all(len(values) >= 3 for values in distance_history.values()):
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
            print("Unknown point (updated):", latest_position)
    else:
        print(f"Invalid distance data received: {data}")

# Servo and laser helper functions
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

def point_laser_at_position(position):
    target_x, target_y, target_z = position
    target_horizontal_angle, target_vertical_angle = cartesian_to_servo_angles(target_x, target_y, target_z)
    target_horizontal_angle -= 20
    target_vertical_angle += 7

    current_horizontal_angle = 0
    current_vertical_angle = 0
    step_size = 1

    turn_laser_on()
    time.sleep(1)

    while round(current_horizontal_angle, 1) != round(target_horizontal_angle, 1):
        if current_horizontal_angle < target_horizontal_angle:
            current_horizontal_angle = min(current_horizontal_angle + step_size, target_horizontal_angle)
        else:
            current_horizontal_angle = max(current_horizontal_angle - step_size, target_horizontal_angle)

        duty = angle_to_duty_cycle(current_horizontal_angle)
        servo_x.ChangeDutyCycle(duty)
        time.sleep(0.1)
        servo_x.ChangeDutyCycle(0)

    while round(current_vertical_angle, 1) != round(target_vertical_angle, 1):
        if current_vertical_angle < target_vertical_angle:
            current_vertical_angle = min(current_vertical_angle + step_size, target_vertical_angle)
        else:
            current_vertical_angle = max(current_vertical_angle - step_size, target_vertical_angle)

        duty = angle_to_duty_cycle(current_vertical_angle)
        servo_y.ChangeDutyCycle(duty)
        time.sleep(0.1)
        servo_y.ChangeDutyCycle(0)

    print("Laser pointing complete.")
    time.sleep(300)

# MQTT setup
mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topics = [
    "esp32/distance/ULCORNER",
    "esp32/distance/URCORNER",
    "esp32/distance/BLCORNER",
    "esp32/distance/BRCORNER"
]

client = mqtt.Client()
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)
for topic in mqtt_topics:
    client.subscribe(topic)

print(f"Subscribed to {mqtt_topics}...")

# Main loop with button monitoring
try:
    while True:
        client.loop(timeout=0.1)  # Non-blocking
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:  # Button pressed
            if latest_position is not None:
                print("Button pressed! Moving servos to latest triangulated position...")
                point_laser_at_position(latest_position)
            else:
                print("Button pressed but position not available yet.")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    servo_x.stop()
    servo_y.stop()
    GPIO.output(LASER_PIN, GPIO.LOW)
    GPIO.cleanup()
    print("GPIO cleaned up. Laser OFF.")
