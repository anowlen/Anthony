#include <HardwareSerial.h>

//Utilizing REYAX RYUW122 UWB module to code

// This is for ANCHOR  mode, or AT+MODE 1
// Each address needs to be unique for each TAG so the anchor can distinguish. 
// Change the name of the address in the sendAddress() command.

HardwareSerial mySerial(1); //Use UART1
// These can be changed later to accomdate a more space conscious design
#define RX_PIN 18 //RX Pin of ESP32 connected to TX Pin of RYUW122
#define TX_PIN 19 //TX Pin of ESP32 connected to RX Pin of RYUW122

unsigned long previousMillis = 0;  // Variable to track last time AT command was sent
const long interval = 5000; 

void clearSerialBuffer(){
  while (Serial.available()){
    Serial.read();
  }
}

bool sendMode(){
  clearSerialBuffer();
  mySerial.print("AT+MODE=1\r\n");
  if(mySerial.available()){
    String response = mySerial.readString();
    Serial.print(response);
    delay(2000);
    return 1;
  }
  else {
    return 0;
  }
}

bool sendNetwork(){
  clearSerialBuffer();
  mySerial.print("AT+NETWORKID=ANTHONY4\r\n");
  if(mySerial.available()){
    String response = mySerial.readString();
    Serial.print(response);
    delay(2000);
    return 1;
  }
  else {
    return 0;
  }
}

bool sendAddress(){
  clearSerialBuffer();
  mySerial.print("AT+ADDRESS=REMOTE15\r\n");
  if(mySerial.available()){
    String response = mySerial.readString();
    Serial.print(response);
    delay(2000);
    return 1;
  }
  else {
    return 0;
  }
}

void setup() {
  // put your setup code here, to run once:
  // Buad rate for ESP32 to communciate with computer
  Serial.begin(115200);

  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("Ready to send at commands to the RYUW122");

  //Confirm Mode command sent.
  bool atModeConfig = sendMode();
  // If nothing comes back into the serial monitor, re-call the command
  while(!atModeConfig){
    Serial.println("Failed to Config Mode");
    delay(1000);
    atModeConfig = sendMode();
  }
  delay(2000);
  Serial.println("Mode Configured");
  //Confirm Network command sent.
  bool atNetworkConfig = sendNetwork();
  while(!atNetworkConfig){
    Serial.println("Failed to Config NetworkID");
    delay(1000);
    atNetworkConfig = sendNetwork();
  }
  delay(2000);
  Serial.println("NetworkID Configured");

  //Confirm address command sent
  bool atAddressConfig = sendAddress();
  while(!atAddressConfig){
    Serial.println("Failed to Config AddressID");
    delay(1000);
    atAddressConfig = sendAddress();
  }
  delay(2000); 
  Serial.println("AddressID Configured");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Check if it's time to send an AT command
  clearSerialBuffer();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    String atCommandUL = "AT+ANCHOR_SEND=ULCORNER,4,TEST\r\n";
    Serial.print("Sending: " + atCommandUL);
    mySerial.print(atCommandUL);
    delay(1000);
    String atCommandUR = "AT+ANCHOR_SEND=URCORNER,4,TEST\r\n";
    Serial.print("Sending: " + atCommandUR);
    mySerial.print(atCommandUR);
    // while(!mySerial.available()){}
    // Serial.println(mySerial.readString());
    // String atCommandUL = "AT+ANCHOR_SEND=LLCORNER,4,TEST\r\n";
    // Serial.print("Sending: " + atCommandUL);
    // mySerial.print(atCommandUL);
    // while(!mySerial.available()){}
    // Serial.println(mySerial.readString());
    // String atCommandUL = "AT+ANCHOR_SEND=URCORNER,4,TEST\r\n";
    // Serial.print("Sending: " + atCommandUL);
    // mySerial.print(atCommandUL);
    // while(!mySerial.available()){}
    // Serial.println(mySerial.readString());
    // String atCommandUL = "AT+ANCHOR_SEND=LRCORNER,4,TEST\r\n";
    // Serial.print("Sending: " + atCommandUL);
    // mySerial.print(atCommandUL);
    // while(!mySerial.available()){}
    // Serial.println(mySerial.readString());
  }
  if (Serial.available()) {
    String command = Serial.readString();
    Serial.print("Manual Input: " + command);
    mySerial.print(command);
  }
  clearSerialBuffer();

  // Forward data from RYUW122 to Serial Monitor
  if (mySerial.available()) {
    String response = mySerial.readString();
    Serial.print("Response: " + response);
  }

}
