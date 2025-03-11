import paho.mqtt.client as mqtt

# The callback function to handle messages received from the broker
def on_message(client, userdata, msg):
    print(f"Received distance: {msg.payload.decode()} on topic {msg.topic}")

# MQTT Broker details
mqtt_broker = "172.20.10.8"  # IP address of the Raspberry Pi, same as in the ESP32 code
mqtt_port = 1883
mqtt_topic = "esp32/distance"  # Topic where the ESP32 publishes distance data

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
