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

ref1 = np.array([-27.94,0,-232.41]) #bottom left
ref2 = np.array([-83.82,461,-232.41]) #upper left
ref3 = np.array([218.44,461,-232.41]) #upper right
ref4 = np.array([299.72,0,-232.41]) #bottom right

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

        # Wait for full history from all anchors
        if all(len(values) >= MAX_HISTORY_LENGTH for values in distance_history.values()) and not servo_ready:
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

            initial_guess = [0.5, 0.5, 0.5]
            result = least_squares(my_system, initial_guess)
            latest_unknown_point = np.round(result.x, decimals=4)
            print("Updated Unknown Point:", latest_unknown_point)

            # Signal servo loop to move
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

    # Wait for all distances to be collected
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

