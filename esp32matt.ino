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
const long interval = 5000;

void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
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
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  reconnect();

  Serial.println("Configuring UWB Module...");
  mySerial.print("AT+MODE=1\r\n");
  delay(500);
  mySerial.print("AT+NETWORKID=ANTHONY4\r\n"); //should be the same
  delay(500);
  mySerial.print("AT+ADDRESS=REMOTE15\r\n");
  delay(500);
  Serial.println("UWB Module Configured.");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    mySerial.print("AT+ANCHOR_SEND=ULCORNER,4,TEST\r\n");
    delay(1000);
    
    if (mySerial.available()) {
      String response = mySerial.readString();
      Serial.println("UWB Response: " + response);
      client.publish("esp32/distance/ULCORNER", response.c_str());
    }

    mySerial.print("AT+ANCHOR_SEND=URCORNER,4,TEST\r\n");
    delay(1000);
    
    if (mySerial.available()) {
      String response = mySerial.readString();
      Serial.println("UWB Response: " + response);
      client.publish("esp32/distance/URCORNER", response.c_str());
    }

    mySerial.print("AT+ANCHOR_SEND=BLCORNER,4,TEST\r\n");
    delay(1000);
    
    if (mySerial.available()) {
      String response = mySerial.readString();
      Serial.println("UWB Response: " + response);
      client.publish("esp32/distance/BLCORNER", response.c_str());
    }

    /*mySerial.print("AT+ANCHOR_SEND=BRCORNER,4,TEST\r\n");
    delay(1000);
    
    if (mySerial.available()) {
      String response = mySerial.readString();
      Serial.println("UWB Response: " + response);
      client.publish("esp32/distance/BRCORNER", response.c_str());
    }
    */
  }
}
