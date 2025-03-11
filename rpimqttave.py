import paho.mqtt.client as mqtt
from collections import deque
import json  

# Store the last 10 values for each of the 4 ESP32s
data_store = {
    "esp32_1": deque(maxlen=10),
    "esp32_2": deque(maxlen=10),
    "esp32_3": deque(maxlen=10),
    "esp32_4": deque(maxlen=10),
}

# Function to calculate and print averages
def calculate_averages():
    averages = {key: sum(data_store[key]) / len(data_store[key]) if data_store[key] else 0 for key in data_store}
    print(f"Averages: {averages}")
    return averages

# The callback function to handle messages received from the broker
def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())  # Parse incoming JSON data
        for key in data_store.keys():
            if key in payload:  # Check if the ESP32 data exists
                data_store[key].append(payload[key])  # Store latest value

        # Calculate and print averages
        calculate_averages()

    except json.JSONDecodeError:
        print(f"Invalid JSON received: {msg.payload.decode()}")

# MQTT Broker details
mqtt_broker = "172.20.10.8"
mqtt_port = 1883
mqtt_topic = "esp32/data"  # Adjust based on actual topic

# Create an MQTT client instance
client = mqtt.Client()

# Set up the callback function for received messages
client.on_message = on_message

# Connect to the broker
client.connect(mqtt_broker, mqtt_port, 60)

# Subscribe to the topic
client.subscribe(mqtt_topic)

# Loop to keep the client connected and receive messages
print(f"Subscribed to {mqtt_topic}...")
client.loop_forever()

