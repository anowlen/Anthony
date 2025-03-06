import requests
from bs4 import BeautifulSoup

# Replace with the IP address of your ESP32
esp32_ip = "http://192.168.1.100"  # Use the actual IP of your ESP32

# Make a GET request to the ESP32 Web Server
try:
    response = requests.get(esp32_ip)
    response.raise_for_status()  # Check if the request was successful (status code 200)

    # Parse the HTML content using BeautifulSoup
    soup = BeautifulSoup(response.text, 'html.parser')

    # Extract GPIO16 and GPIO17 statuses from the HTML content
    gpio16_status = "ON" if "GPIO16 LED Status: ON" in soup.text else "OFF"
    gpio17_status = "ON" if "GPIO17 LED Status: ON" in soup.text else "OFF"

    # Print out the current statuses of GPIO16 and GPIO17
    print(f"GPIO16 Status: {gpio16_status}")
    print(f"GPIO17 Status: {gpio17_status}")

except requests.exceptions.RequestException as e:
    print(f"Error making GET request to ESP32 Web Server: {e}")

