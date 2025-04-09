import paho.mqtt.client as mqtt
import re  # Import regex module for extracting numbers
import  numpy as np
from scipy.optimize import least_squares


# Dictionary to store the most recent 5 values for each ESP32
distance_history = {
    "ULCORNER": [],
    "URCORNER": [],
    "BLCORNER": [],
    "BRCORNER": []
}

MAX_HISTORY_LENGTH = 10

# Function to update the history and maintain only the last 5 values
def update_history(esp32_id, new_distance):
    if esp32_id in distance_history:
        distance_history[esp32_id].append(new_distance)  # Add new value
        if len(distance_history[esp32_id]) > MAX_HISTORY_LENGTH:  # Keep only the last 5 values
            distance_history[esp32_id].pop(0)

# Function to extract the distance from the received message
def extract_distance(data):
    match = re.search(r'(\d+)\s*cm', data)  # Find number before "cm"
    return float(match.group(1)) if match else None  # Convert to float

# Callback for when a message is received
def on_message(client, userdata, msg):
    esp32_id = msg.topic.split("/")[-1]
    data = msg.payload.decode().strip()

    distance = extract_distance(data)

    if distance is not None:
        update_history(esp32_id, distance)

        print("\nUpdated Distance History:")
        for key, values in distance_history.items():
            print(f"{key}: {values}")

        # Check if all corners have at least 3 values before triangulation
        if all(len(values) >= 3 for values in distance_history.values()):
            # Calculate averages
            d1 = calculate_average(distance_history["BLCORNER"])
            d2 = calculate_average(distance_history["ULCORNER"])
            d3 = calculate_average(distance_history["URCORNER"])
            d4 = calculate_average(distance_history["BRCORNER"])

            # Triangulation function
            def my_system(vars):
                x, y, z = vars
                F = np.zeros(4)
                F[0] = np.sqrt((x - ref1[0])**2 + (y - ref1[1])**2 + (z - ref1[2])**2) - d1
                F[1] = np.sqrt((x - ref2[0])**2 + (y - ref2[1])**2 + (z - ref2[2])**2) - d2
                F[2] = np.sqrt((x - ref3[0])**2 + (y - ref3[1])**2 + (z - ref3[2])**2) - d3
                F[3] = np.sqrt((x - ref4[0])**2 + (y - ref4[1])**2 + (z - ref4[2])**2) - d4
                return F

            initial_guess = [100, 100, 50]  # Better starting point than (0.5, 0.5, 0.5)
            result = least_squares(my_system, initial_guess)
            unknown_point = np.round(result.x, decimals=4)

            print("Unknown point (updated):", unknown_point)

    else:
        print(f"Invalid distance data received: {data}")

def calculate_average(arr):
    arr_np = np.array(arr)
    median = np.median(arr_np)
    # Keep values within ±1 standard deviation of the median
    std = np.std(arr_np)
    filtered = arr_np[(arr_np >= median - std) & (arr_np <= median + std)]
    if len(filtered) == 0:
        return median
    return np.mean(filtered)


###TRIANGULATION CODE

# Define reference points #this makes a rectangle, we have to change this later
ref1 = np.array([-157.5, 0, -233.7])   # bottom-left
ref2 = np.array([-657.86, 955.0, -137.16])   # top-left
ref3 = np.array([274.3, 492.8, -149.9])   # top-right
ref4 = np.array([274.3, 67.31, -149.9])   # bottom-right




####




# MQTT Broker details
mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topics = ["esp32/distance/ULCORNER", "esp32/distance/URCORNER", "esp32/distance/BLCORNER","esp32/distance/BRCORNER"]

# Create an MQTT client
client = mqtt.Client()
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)

# Subscribe to all relevant topics
for topic in mqtt_topics:
    client.subscribe(topic)

print(f"Subscribed to {mqtt_topics}...")
client.loop_forever()

