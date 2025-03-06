import bluetooth

def receive_bluetooth_data():
    server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server_socket.bind(("", bluetooth.PORT_ANY))
    server_socket.listen(1)

    port = server_socket.getsockname()[1]
    bluetooth.advertise_service(server_socket, "ESP32Receiver",
                                service_classes=[bluetooth.SERIAL_PORT_CLASS],
                                profiles=[bluetooth.SERIAL_PORT_PROFILE])

    print(f"Waiting for Bluetooth connection on RFCOMM channel {port}...")

    client_socket, client_info = server_socket.accept()
    print(f"Connected to {client_info}")

    try:
        while True:
            data = client_socket.recv(1024)  # Receive data (max 1024 bytes)
            if not data:
                break
            print(f"Received: {data.decode('utf-8')}")  # Decode and print data
    except OSError:
        pass

    print("Connection closed.")
    client_socket.close()
    server_socket.close()

#if name == "main":
    receive_bluetooth_data()
