import paho.mqtt.client as mqtt
import re  # Import regex module for extracting numbers

# Dictionary to store the most recent 5 values for each ESP32
distance_history = {
    "ULCORNER": [],
    "URCORNER": [],
    "BLCORNER": []
}

# Function to update the history and maintain only the last 5 values
def update_history(esp32_id, new_distance):
    if esp32_id in distance_history:
        distance_history[esp32_id].append(new_distance)  # Add new value
        if len(distance_history[esp32_id]) > 5:  # Keep only the last 5 values
            distance_history[esp32_id].pop(0)

# Function to extract the distance from the received message
def extract_distance(data):
    match = re.search(r'(\d+)\s*cm', data)  # Find number before "cm"
    return float(match.group(1)) if match else None  # Convert to float

# Callback for when a message is received
def on_message(client, userdata, msg):
    esp32_id = msg.topic.split("/")[-1]  # Extract "ULCORNER", "URCORNER", etc.
    data = msg.payload.decode().strip()  # Remove extra whitespace

    distance = extract_distance(data)  # Extract only the distance value

    if distance is not None:
        update_history(esp32_id, distance)
        
        # Print the most recent 5 values for each ESP32
        print("\nUpdated Distance History:")
        for key, values in distance_history.items():
            print(f"{key}: {values}")
    else:
        print(f"Invalid distance data received: {data}")

# MQTT Broker details
mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topics = ["esp32/distance/ULCORNER", "esp32/distance/URCORNER", "esp32/distance/BLCORNER"]

# Create an MQTT client
client = mqtt.Client()
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)

# Subscribe to all relevant topics
for topic in mqtt_topics:
    client.subscribe(topic)

print(f"Subscribed to {mqtt_topics}...")
client.loop_forever()

