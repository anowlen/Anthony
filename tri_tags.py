import paho.mqtt.client as mqtt
import re
import numpy as np
from scipy.optimize import fsolve

# Dictionary to store the most recent 10 values for each ESP32
distance_history = {
    "ULCORNER": [],
    "URCORNER": [],
    "BLCORNER": [],
    "BRCORNER": []
}
window_size = 10  # Define window size for moving average

def update_history(esp32_id, new_distance):
    if esp32_id in distance_history:
        distance_history[esp32_id].append(new_distance)
        if len(distance_history[esp32_id]) > window_size:
            distance_history[esp32_id].pop(0)

def extract_distance(data):
    match = re.search(r'(\d+)\s*cm', data)
    return float(match.group(1)) if match else None

def remove_outliers(window):
    Q1 = np.percentile(window, 25)
    Q3 = np.percentile(window, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    return [x for x in window if lower_bound < x < upper_bound]

def moving_average_with_outlier_removal(data):
    if len(data) < window_size:
        return np.nan  # Not enough data yet
    filtered_window = remove_outliers(data[-window_size:])
    return np.mean(filtered_window) if filtered_window else np.nan

def on_message(client, userdata, msg):
    esp32_id = msg.topic.split("/")[-1]
    data = msg.payload.decode().strip()
    distance = extract_distance(data)
    if distance is not None:
        update_history(esp32_id, distance)
        moving_averages = {key: moving_average_with_outlier_removal(values) for key, values in distance_history.items()}
        print("Updated Moving Averages:", moving_averages)
        solve_position(moving_averages)
    else:
        print(f"Invalid distance data received: {data}")

def solve_position(moving_averages):
    if any(np.isnan(val) for val in moving_averages.values()):
        return  # Wait until all values are available
    
    def my_system(vars):
        x, y, z = vars
        l, w = 1, 1  # Dimensions of the setup
        d1, d2, d3, d4 = moving_averages.values()
        return [
            np.sqrt(x**2 + y**2 + z**2) - d1,
            np.sqrt(x**2 + (y - w)**2 + z**2) - d2,
            np.sqrt((x + l)**2 + y**2 + z**2) - d3,
            np.sqrt((x + l)**2 + (y + w)**2 + z**2) - d4
        ]
    
    initial_guess = [0.5, 0.5, 0.5]
    solution = fsolve(my_system, initial_guess)
    print("Calculated Position:", np.round(solution, 3))

mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topics = ["esp32/distance/ULCORNER", "esp32/distance/URCORNER", "esp32/distance/BLCORNER", "esp32/distance/BRCORNER"]

client = mqtt.Client()
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)

for topic in mqtt_topics:
    client.subscribe(topic)

print(f"Subscribed to {mqtt_topics}...")
client.loop_forever()

