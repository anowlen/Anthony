#include <HardwareSerial.h>

// Utilizing REYAX RYUW122 UWB module
// This is for ANCHOR mode (AT+MODE=1)
// Each address needs to be unique for each TAG so the anchor can distinguish.

HardwareSerial mySerial(1); // Use UART1

#define RX_PIN 18 // RX Pin of ESP32 connected to TX Pin of RYUW122
#define TX_PIN 19 // TX Pin of ESP32 connected to RX Pin of RYUW122

unsigned long previousMillis = 0;
const long interval = 1000; // 1 second
int anchor_index = 0; // Track which anchor to send next

String anchors[] = {"ULCORNER", "URCORNER", "BLCORNER", "BRCORNER"};

void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();
  }
}

bool sendMode() {
  clearSerialBuffer();
  mySerial.print("AT+MODE=1\r\n");
  if (mySerial.available()) {
    String response = mySerial.readString();
    Serial.print(response);
    delay(2000);
    return true;
  } else {
    return false;
  }
}

bool sendNetwork() {
  clearSerialBuffer();
  mySerial.print("AT+NETWORKID=ANTHONY4\r\n");
  if (mySerial.available()) {
    String response = mySerial.readString();
    Serial.print(response);
    delay(2000);
    return true;
  } else {
    return false;
  }
}

bool sendAddress() {
  clearSerialBuffer();
  mySerial.print("AT+ADDRESS=REMOTE15\r\n");
  if (mySerial.available()) {
    String response = mySerial.readString();
    Serial.print(response);
    delay(2000);
    return true;
  } else {
    return false;
  }
}

void setup() {
  // Setup code to run once
  Serial.begin(115200); // ESP32 to computer
  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // ESP32 to UWB

  Serial.println("Ready to send AT commands to the RYUW122");

  // Confirm Mode command
  bool atModeConfig = sendMode();
  while (!atModeConfig) {
    Serial.println("Failed to Config Mode");
    delay(1000);
    atModeConfig = sendMode();
  }
  delay(2000);
  Serial.println("Mode Configured");

  // Confirm Network ID
  bool atNetworkConfig = sendNetwork();
  while (!atNetworkConfig) {
    Serial.println("Failed to Config NetworkID");
    delay(1000);
    atNetworkConfig = sendNetwork();
  }
  delay(2000);
  Serial.println("NetworkID Configured");

  // Confirm Address ID
  bool atAddressConfig = sendAddress();
  while (!atAddressConfig) {
    Serial.println("Failed to Config AddressID");
    delay(1000);
    atAddressConfig = sendAddress();
  }
  delay(2000);
  Serial.println("AddressID Configured");
}

void loop() {
  clearSerialBuffer();
  unsigned long currentMillis = millis();

  // Send one AT+ANCHOR_SEND command every 1 second
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Prepare and send the AT command
    String atCommand = "AT+ANCHOR_SEND=" + anchors[anchor_index] + ",4,TEST\r\n";
    Serial.print("Sending: " + atCommand);
    mySerial.print(atCommand);

    // Move to next anchor (cycle through 0-3)
    anchor_index = (anchor_index + 1) % 4;
  }

  // Manual Serial input forwarding
  if (Serial.available()) {
    String command = Serial.readString();
    Serial.print("Manual Input: " + command);
    mySerial.print(command);
  }

  // Forward UWB module responses to Serial monitor
  if (mySerial.available()) {
    String response = mySerial.readString();
    Serial.print("Response: " + response);
  }
}
