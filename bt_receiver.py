import bluetooth

server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_socket.bind(("", bluetooth.PORT_ANY))
server_socket.listen(1)

port = server_socket.getsockname()[1]
bluetooth.advertise_service(server_socket, "RPiReceiver",
                            service_classes=[bluetooth.SERIAL_PORT_CLASS],
                            profiles=[bluetooth.SERIAL_PORT_PROFILE])

print(f"Waiting for connection on RFCOMM channel {port}...")

client_socket, client_info = server_socket.accept()
print(f"Accepted connection from {client_info}")

try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        print(f"Received: {data.decode('utf-8')}")
except OSError:
    pass

print("Connection closed.")
client_socket.close()
server_socket.close()
