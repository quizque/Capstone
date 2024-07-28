import serial
import socket
from threading import Thread
from queue import Queue
import struct

# Serial port configuration
serial_port = "COM5"  # Update this with your serial port
baud_rate = 115200

# Socket configuration
host = "localhost"  # Update this with your desired host
port = 12345  # Update this with your desired port

# Thread safe queuing of serial data
serial_data_queue = Queue()

connected_clients = []


def serial_thread():

    # Open the serial port
    ser = serial.Serial(serial_port, baud_rate)

    try:
        while True:
            # Read a line from the serial port
            ser.reset_input_buffer()
            line = ser.readline().decode("utf-8").strip()
            # print(f"Received: {line}")
            r = line.split(",")
            if len(r) == 7:
                r.append(0)
                r.append(0)
            if len(r) != 9:
                continue

            # print(r)

            # Parse the quaternion
            q = list(map(float, r))

            # Pack the list of floats (quaternion data) into a byte string
            data_bytes = struct.pack("fffffffff", *q)

            # Send the data to every connected client
            for client_socket in connected_clients:
                try:
                    if client_socket.fileno() != -1:
                        client_socket.sendall(data_bytes)
                except:
                    print(f"Client {client_socket.getpeername()} disconnected.")
                    connected_clients.remove(client_socket)

            if serial_data_queue.qsize() > 1:
                serial_data_queue.get()

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        ser.close()


def accept_clients(server_socket):
    try:
        while True:
            # Accept a client connection
            client_socket, address = server_socket.accept()
            print(f"Accepted connection from {address}")

            connected_clients.append(client_socket)

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        server_socket.close()


def main():
    # Create a socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the host and port
    server_socket.bind((host, port))

    # Listen for incoming connections
    server_socket.listen()

    # Start accepting clients in a separate thread
    accept_thread = Thread(target=accept_clients, args=(server_socket,))
    accept_thread.daemon = True
    accept_thread.start()

    serial_thread()


if __name__ == "__main__":
    main()
