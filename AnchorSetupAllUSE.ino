#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

// ==== WiFi and MQTT Settings ====
const char* ssid = "Alanna";
const char* password = "alannahotspot";
const char* mqtt_server = "172.20.10.8";

WiFiClient espClient;
PubSubClient client(espClient);

// ==== UWB Serial Settings ====
HardwareSerial mySerial(1);
#define RX_PIN 18
#define TX_PIN 19

// ==== Timing and Anchors ====
unsigned long previousMillis = 0;
const long interval = 1000; // 1 second

String anchors[] = {"ULCORNER", "URCORNER", "BLCORNER", "BRCORNER"};
String topics[]  = {"esp32/distance/ULCORNER", "esp32/distance/URCORNER", "esp32/distance/BLCORNER", "esp32/distance/BRCORNER"};
int anchor_index = 0; // Track which anchor to send next

// ==== WiFi Setup ====
void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// ==== MQTT Reconnect ====
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
      delay(3000);
    }
  }
}

// ==== Send AT command to UWB, read response, publish ====
void sendCommandAndPublish(String command, String topic) {
  mySerial.flush(); // Clear Serial buffer
  mySerial.print(command);

  delay(100); // Small delay to wait for UWB response
  
  String response = "";
  unsigned long start = millis();
  while (millis() - start < 200) { // Give 200ms to receive response
    while (mySerial.available()) {
      response += mySerial.readStringUntil('\n');
    }
  }

  if (response.length() > 0) {
    Serial.println("UWB Response: " + response);
    client.publish(topic.c_str(), response.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // ==== UWB Module Configuration ====
  Serial.println("Configuring UWB Module...");
  mySerial.print("AT+MODE=1\r\n");
  delay(200);
  mySerial.print("AT+NETWORKID=ANTHONY4\r\n");
  delay(200);
  mySerial.print("AT+ADDRESS=REMOTE15\r\n");
  delay(200);
  Serial.println("UWB Module Configured.");

  // ==== WiFi and MQTT Setup ====
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  reconnect();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Build AT command
    String atCommand = "AT+ANCHOR_SEND=" + anchors[anchor_index] + ",4,TEST\r\n";
    String mqttTopic = topics[anchor_index];

    Serial.println("Sending: " + atCommand);
    sendCommandAndPublish(atCommand, mqttTopic);

    // Move to next anchor
    anchor_index = (anchor_index + 1) % 4;
  }
}
