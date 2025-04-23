#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

// WiFi and MQTT Settings
const char* ssid = "Alanna";
const char* password = "alannahotspot";
const char* mqtt_server = "172.20.10.8";

WiFiClient espClient;
PubSubClient client(espClient);

// UWB Module Serial Communication
HardwareSerial mySerial(1);
#define RX_PIN 18
#define TX_PIN 19

unsigned long previousMillis = 0;
const long interval = 250;  // now 250ms

void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

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

void sendCommandAndPublish(const char* command, const char* topic) {
  mySerial.flush();  // clear buffer
  mySerial.print(command);
  delay(50); // short delay to allow response

  unsigned long start = millis();
  String response = "";

  while ((millis() - start) < 150 && mySerial.available()) {
    response += mySerial.readStringUntil('\n');
  }

  if (response.length() > 0) {
    Serial.println("UWB Response: " + response);
    client.publish(topic, response.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("Configuring UWB Module...");
  mySerial.print("AT+MODE=1\r\n");
  delay(200);
  mySerial.print("AT+NETWORKID=ANTHONY4\r\n");
  delay(200);
  mySerial.print("AT+ADDRESS=REMOTE15\r\n");
  delay(200);
  Serial.println("UWB Module Configured.");

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

    sendCommandAndPublish("AT+ANCHOR_SEND=ULCORNER,4,TEST\r\n", "esp32/distance/ULCORNER");
    sendCommandAndPublish("AT+ANCHOR_SEND=URCORNER,4,TEST\r\n", "esp32/distance/URCORNER");
    sendCommandAndPublish("AT+ANCHOR_SEND=BLCORNER,4,TEST\r\n", "esp32/distance/BLCORNER");
    sendCommandAndPublish("AT+ANCHOR_SEND=BRCORNER,4,TEST\r\n", "esp32/distance/BRCORNER");
  }
}
