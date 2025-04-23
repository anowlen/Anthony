import paho.mqtt.client as mqtt
import re
import numpy as np
from scipy.optimize import least_squares
import RPi.GPIO as GPIO
import time
import math

# === Globals ===
latest_position = None
current_horizontal_angle = 0
current_vertical_angle = 0

# === GPIO setup ===
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # Suppress "channel already in use" warnings

SERVO_X_PIN = 18
SERVO_Y_PIN = 17
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

# === Distance history ===
distance_history = {
    "ULCORNER": [],
    "URCORNER": [],
    "BLCORNER": [],
    "BRCORNER": []
}
MAX_HISTORY_LENGTH = 10

# === Reference points ===
ref1 = np.array([-27.94, 0, -232.41])     # BL
ref2 = np.array([-83.82, 461, -232.41])   # UL
ref3 = np.array([218.44, 461, -232.41])   # UR
ref4 = np.array([299.72, 0, -134.62])     # BR

# === MQTT Broker Info ===
mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topics = [
    "esp32/distance/ULCORNER",
    "esp32/distance/URCORNER",
    "esp32/distance/BLCORNER",
    "esp32/distance/BRCORNER"
]

# === Helper functions ===
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

        print(f"\n[DEBUG] Triangulation @ {time.strftime('%H:%M:%S')}")
        print(f"  BLCORNER: {d1:.2f} cm")
        print(f"  ULCORNER: {d2:.2f} cm")
        print(f"  URCORNER: {d3:.2f} cm")
        print(f"  BRCORNER: {d4:.2f} cm")

        def my_system(vars):
            x, y, z = vars
            F = np.zeros(4)
            F[0] = np.sqrt((x - ref1[0])**2 + (y - ref1[1])**2 + (z - ref1[2])**2) - d1
            F[1] = np.sqrt((x - ref2[0])**2 + (y - ref2[1])**2 + (z - ref2[2])**2) - d2
            F[2] = np.sqrt((x - ref3[0])**2 + (y - ref3[1])**2 + (z - ref3[2])**2) - d3
            F[3] = np.sqrt((x - ref4[0])**2 + (y - ref4[1])**2 + (z - ref4[2])**2) - d4
            return F

        initial_guess = [100, 100, 100]
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
    ratio = z / p if p != 0 else 0
    ratio = max(min(ratio, 10), -10)
    vertical_angle = -90 - math.degrees(math.atan(ratio))
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

    print(f"[DEBUG] Moving horizontal to: {target_horizontal_angle:.2f}")
    while round(current_horizontal_angle, 1) != round(target_horizontal_angle, 1):
        if current_horizontal_angle < target_horizontal_angle:
            current_horizontal_angle = min(current_horizontal_angle + step_size, target_horizontal_angle)
        else:
            current_horizontal_angle = max(current_horizontal_angle - step_size, target_horizontal_angle)

        duty = angle_to_duty_cycle(current_horizontal_angle)
        print(f"[DEBUG] Horizontal angle: {current_horizontal_angle:.1f} -> Duty: {duty}")
        servo_x.ChangeDutyCycle(duty)
        time.sleep(0.1)
        servo_x.ChangeDutyCycle(0)
        time.sleep(0.1)

    time.sleep(0.3)

    print(f"[DEBUG] Moving vertical to: {target_vertical_angle:.2f}")
    while round(current_vertical_angle, 1) != round(target_vertical_angle, 1):
        if current_vertical_angle < target_vertical_angle:
            current_vertical_angle = min(current_vertical_angle + step_size, target_vertical_angle)
        else:
            current_vertical_angle = max(current_vertical_angle - step_size, target_vertical_angle)

        duty = angle_to_duty_cycle(current_vertical_angle)
        print(f"[DEBUG] Vertical angle: {current_vertical_angle:.1f} -> Duty: {duty}")
        servo_y.ChangeDutyCycle(duty)
        time.sleep(0.1)
        servo_y.ChangeDutyCycle(0)
        time.sleep(0.1)

    print("[DEBUG] Finished vertical movement.")
    time.sleep(10)
    turn_laser_off()


# === MQTT setup ===
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
            time.sleep(0.5)  # Debounce

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    servo_x.stop()
    servo_y.stop()
    turn_laser_off()
    GPIO.cleanup()
    print("GPIO cleaned up.")
