# Hosts a TCP server which sends out all of the data received by a device
# This makes it a lot easier to design programs and recover from a crash
# since I don't have to wait for the COM port connect times and windows
# freaking out because I disconnected something

import serial
import socket
from threading import Thread
from queue import Queue
import struct

# Serial port configuration
serial_port = "COM5" 
baud_rate = 115200

# Socket configuration
host = "localhost"
port = 12345

# Thread safe queuing of serial data
serial_data_queue = Queue()

# Store TCP clients
connected_clients = []

# Serial thread
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

            # Parse the string
            q = list(map(float, r))

            # Pack the list of floats into a byte string
            data_bytes = struct.pack("fffffffff", *q)

            # Send the data to every connected client
            for client_socket in connected_clients:
                try:
                    # Check to make sure client exists
                    if client_socket.fileno() != -1:
                        client_socket.sendall(data_bytes)
                except:
                    print(f"Client {client_socket.getpeername()} disconnected.")
                    connected_clients.remove(client_socket)

            # If there is extra data in the queue, remove it
            if serial_data_queue.qsize() > 1:
                serial_data_queue.get()

    # Since this is the main thread, we need a way to stop it
    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        ser.close()

# TCP client handling thread
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
    accept_thread.daemon = True # This will make it stop when we press control-c
    accept_thread.start()

    # Run main thread
    serial_thread()


if __name__ == "__main__":
    main()
