import RPi.GPIO as GPIO
import time
import math
import re
import numpy as np
from scipy.optimize import least_squares
import paho.mqtt.client as mqtt

# --- GPIO Setup ---
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

# --- Angle Conversion ---
def cartesian_to_servo_angles(x, y, z):
    horizontal_angle = -(math.degrees(math.atan2(x, y)) - 180) - 180
    p = math.sqrt(x**2 + y**2)
    vertical_angle = -90 - math.degrees(math.atan(z / p))
    return horizontal_angle - 20, vertical_angle + 7  # Apply offsets here

def angle_to_duty_cycle(angle):
    return max(2, min(12, 2 + (angle + 90) * 10 / 180))

def turn_laser_on():
    GPIO.output(LASER_PIN, GPIO.HIGH)
    print("Laser ON")

def move_laser_to(x, y, z):
    h_angle, v_angle = cartesian_to_servo_angles(x, y, z)

    step = 1
    global current_horizontal_angle, current_vertical_angle

    while abs(current_horizontal_angle - h_angle) >= 0.1:
        current_horizontal_angle += step if current_horizontal_angle < h_angle else -step
        servo_x.ChangeDutyCycle(angle_to_duty_cycle(current_horizontal_angle))
        print(f"Horizontal: {current_horizontal_angle:.2f}°")
        time.sleep(0.1)

    while abs(current_vertical_angle - v_angle) >= 0.1:
        current_vertical_angle += step if current_vertical_angle < v_angle else -step
        servo_y.ChangeDutyCycle(angle_to_duty_cycle(current_vertical_angle))
        print(f"Vertical: {current_vertical_angle:.2f}°")
        time.sleep(0.1)

# --- Triangulation ---
ref1 = np.array([-157.5, 0, -233.7])     # BLCORNER
ref2 = np.array([-657.86, 955.0, -137.16]) # ULCORNER
ref3 = np.array([274.3, 492.8, -149.9])    # URCORNER
ref4 = np.array([274.3, 67.31, -149.9])    # BRCORNER

distance_history = {
    "ULCORNER": [],
    "URCORNER": [],
    "BLCORNER": [],
    "BRCORNER": [],
}
MAX_HISTORY_LENGTH = 10
last_update_time = time.time()

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

def triangulate_and_move():
    d1 = calculate_average(distance_history["BLCORNER"])
    d2 = calculate_average(distance_history["ULCORNER"])
    d3 = calculate_average(distance_history["URCORNER"])
    d4 = calculate_average(distance_history["BRCORNER"])

    def my_system(vars):
        x, y, z = vars
        return [
            np.linalg.norm([x - ref1[0], y - ref1[1], z - ref1[2]]) - d1,
            np.linalg.norm([x - ref2[0], y - ref2[1], z - ref2[2]]) - d2,
            np.linalg.norm([x - ref3[0], y - ref3[1], z - ref3[2]]) - d3,
            np.linalg.norm([x - ref4[0], y - ref4[1], z - ref4[2]]) - d4,
        ]

    result = least_squares(my_system, [100, 100, 50])
    x, y, z = np.round(result.x, 4)
    print("New triangulated point:", (x, y, z))
    move_laser_to(x, y, z)

def on_message(client, userdata, msg):
    global last_update_time
    esp32_id = msg.topic.split("/")[-1]
    distance = extract_distance(msg.payload.decode().strip())

    if distance is not None:
        update_history(esp32_id, distance)
        print(f"{esp32_id} updated: {distance_history[esp32_id]}")

        if all(len(v) >= 3 for v in distance_history.values()):
            now = time.time()
            if now - last_update_time >= 15:
                triangulate_and_move()
                last_update_time = now
    else:
        print(f"Invalid data from {esp32_id}: {msg.payload}")

# --- MQTT Setup ---
mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topics = ["esp32/distance/ULCORNER", "esp32/distance/URCORNER",
               "esp32/distance/BLCORNER", "esp32/distance/BRCORNER"]

client = mqtt.Client()
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)
for topic in mqtt_topics:
    client.subscribe(topic)

# --- Run ---
current_horizontal_angle = 0
current_vertical_angle = 0

try:
    turn_laser_on()
    print("Starting MQTT loop...")
    client.loop_forever()

finally:
    servo_x.stop()
    servo_y.stop()
    GPIO.output(LASER_PIN, GPIO.LOW)
    GPIO.cleanup()
    print("Laser OFF. GPIO cleaned.")

