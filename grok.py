# Raspberry Pi LED Status Reader
import requests
import time
import json

# Replace with your ESP32's IP address (get this from ESP32 Serial Monitor)
ESP32_IP = "172.20.10.4"  # Update this with your actual ESP32 IP
BASE_URL = f"http://{ESP32_IP}"

# Endpoints from your ESP32 code
STATUS_CHECK_URL = BASE_URL  # Root URL shows status in HTML

def get_led_status():
    try:
        # Send GET request to ESP32 root URL
        response = requests.get(STATUS_CHECK_URL, timeout=5)
        
        if response.status_code == 200:
            # Parse the HTML response to extract LED states
            html_content = response.text
            
            # Initialize status dictionary
            led_status = {
                "GPIO16": "Unknown",
                "GPIO17": "Unknown"
            }
            
            # Check GPIO16 status
            if "GPIO16 LED Status: ON" in html_content:
                led_status["GPIO16"] = "ON"
            elif "GPIO16 LED Status: OFF" in html_content:
                led_status["GPIO16"] = "OFF"
                
            # Check GPIO17 status
            if "GPIO17 LED Status: ON" in html_content:
                led_status["GPIO17"] = "ON"
            elif "GPIO17 LED Status: OFF" in html_content:
                led_status["GPIO17"] = "OFF"
                
            return led_status
        else:
            print(f"Error: Status code {response.status_code}")
            return None
            
    except requests.exceptions.RequestException as e:
        print(f"Connection error: {e}")
        return None

def main():
    print("Starting LED status monitoring...")
    while True:
        status = get_led_status()
        if status:
            print(f"LED Status - GPIO16: {status['GPIO16']}, GPIO17: {status['GPIO17']}")
        else:
            print("Failed to get LED status")
        
        # Wait 2 seconds before next check
        time.sleep(2)

if __name__ == "__main__":
    main()
