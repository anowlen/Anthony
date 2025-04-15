import RPi.GPIO as GPIO
import time
import math
import numpy as np
import paho.mqtt.client as mqtt
import re
from scipy.optimize import least_squares
import threading

# === GPIO & PWM Setup ===
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

current_horizontal_angle = 0
current_vertical_angle = 0
latest_unknown_point = None

# === Movement Trigger ===
servo_ready = False
trigger_servo = threading.Event()

# === Helper Functions ===
def angle_to_duty_cycle(angle):
    return max(2, min(12, 2 + (angle + 90) * 10 / 180))

def turn_laser_on():
    GPIO.output(LASER_PIN, GPIO.HIGH)
    print("Laser ON")

def cartesian_to_servo_angles(x, y, z):
    horizontal_angle = -(math.degrees(math.atan2(x, y)) - 180) - 180
    p = math.sqrt(x ** 2 + y ** 2)
    vertical_angle = -90 - math.degrees(math.atan(z / p))
    return horizontal_angle, vertical_angle

# === Distance History & Triangulation ===
distance_history = {
    "ULCORNER": [],
    "URCORNER": [],
    "BLCORNER": [],
    "BRCORNER": [],
}
MAX_HISTORY_LENGTH = 10

ref_lookup = {
    "BLCORNER": np.array([-528.3, 0, -233.7]),
    "ULCORNER": np.array([-528.3, 482.6, -233.7]),
    "URCORNER": np.array([-203.2, 482.6, -233.7]),
    "BRCORNER": np.array([203.2, 0, -233.7]),
}

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

def on_message(client, userdata, msg):
    global latest_unknown_point, servo_ready
    esp32_id = msg.topic.split("/")[-1]
    data = msg.payload.decode().strip()
    distance = extract_distance(data)

    if distance is not None:
        print(f"[{esp32_id}] Received distance: {distance} cm")
        update_history(esp32_id, distance)

        if all(len(values) >= MAX_HISTORY_LENGTH for values in distance_history.values()) and not servo_ready:
            # Compute smoothed averages
            averages = {
                anchor: calculate_average(history)
                for anchor, history in distance_history.items()
            }

            # Drop the furthest anchor
            furthest_anchor = max(averages, key=averages.get)
            print(f"Ignoring {furthest_anchor} (distance = {averages[furthest_anchor]} cm)")

            # Prepare filtered anchors
            used_anchors = [k for k in averages if k != furthest_anchor]
            refs = [ref_lookup[k] for k in used_anchors]
            dists = [averages[k] for k in used_anchors]

            # Solve with 3 anchors
            def my_system(vars):
                x, y, z = vars
                return [
                    np.linalg.norm([x - ref[0], y - ref[1], z - ref[2]]) - dist
                    for ref, dist in zip(refs, dists)
                ]

            initial_guess = [0.5, 0.5, 0.5]
            result = least_squares(my_system, initial_guess)
            latest_unknown_point = np.round(result.x, 4)
            print("Updated Unknown Point:", latest_unknown_point)

            # Trigger servo once
            servo_ready = True
            trigger_servo.set()
    else:
        print(f"Invalid distance data: {data}")

# === MQTT Setup ===
mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topics = [
    "esp32/distance/ULCORNER",
    "esp32/distance/URCORNER",
    "esp32/distance/BLCORNER",
    "esp32/distance/BRCORNER",
]
client = mqtt.Client()
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)
for topic in mqtt_topics:
    client.subscribe(topic)
client.loop_start()

# === Servo Loop (waits for trigger, then moves once) ===
def servo_loop():
    global current_horizontal_angle, current_vertical_angle
    trigger_servo.wait()

    if latest_unknown_point is not None:
        x, y, z = latest_unknown_point
        h_angle, v_angle = cartesian_to_servo_angles(x, y, z)
        h_angle -= 20
        v_angle += 7

        while abs(current_horizontal_angle - h_angle) > 0.1:
            if current_horizontal_angle < h_angle:
                current_horizontal_angle += 1
            else:
                current_horizontal_angle -= 1
            servo_x.ChangeDutyCycle(angle_to_duty_cycle(current_horizontal_angle))
            time.sleep(0.05)

        while abs(current_vertical_angle - v_angle) > 0.1:
            if current_vertical_angle < v_angle:
                current_vertical_angle += 1
            else:
                current_vertical_angle -= 1
            servo_y.ChangeDutyCycle(angle_to_duty_cycle(current_vertical_angle))
            time.sleep(0.05)

        print(f"Moved to: {latest_unknown_point}")

# === Start Threads & Run Forever ===
turn_laser_on()
threading.Thread(target=servo_loop, daemon=True).start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting gracefully...")
    servo_x.stop()
    servo_y.stop()
    GPIO.output(LASER_PIN, GPIO.LOW)
    GPIO.cleanup()

