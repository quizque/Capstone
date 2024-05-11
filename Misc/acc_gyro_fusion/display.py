import serial
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import time

# Adjust these variables based on your serial port and baud rate
SERIAL_PORT = 'COM4'
BAUD_RATE = 115200

def read_serial_data(serial_connection):
    """
    Read a line from the serial connection and extract roll, pitch, and yaw angles.
    """
    try:
        line = serial_connection.readline().decode('utf-8')
        serial_connection.flushInput()
        parts = line.split()
        if len(parts) >= 6:  # Expected at least: "Angle", "X:", value, "Y:", value
            x_angle = float(parts[1])
            y_angle = float(parts[3])
            z_angle = float(parts[5])
            return x_angle, y_angle, z_angle
    except Exception as e:
        print("Error reading serial data:", e)
    return None, None, None

def update_arrow(ax, roll, pitch, yaw):
    """
    Update the arrow in the 3D plot to correspond to the current angles.
    """
    ax.cla()  # Clear the axes for the new plot
    ax.quiver(0, 0, 0, 1, 0, 0, length=1.0, color='r')  # X axis in red
    ax.quiver(0, 0, 0, 0, 1, 0, length=1.0, color='g')  # Y axis in green
    ax.quiver(0, 0, 0, 0, 0, 1, length=1.0, color='b')  # Z axis in blue

    # Convert angles from degrees to radians
    roll_rad, pitch_rad, yaw_rad = np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)

    # Assuming Z value (yaw) is fixed, modify this as needed
    arrow_dir = np.array([np.cos(pitch_rad) * np.cos(yaw_rad),
                          np.cos(pitch_rad) * np.sin(yaw_rad),
                          np.sin(pitch_rad)])

    ax.quiver(0, 0, 0, *arrow_dir, length=0.8, color='k')  # Arrow showing the direction

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.draw()
    plt.pause(1/60)  # Update at 60 FPS

# Initialize serial connection
serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Initialize 3D plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
update_arrow(ax, 0, 0, 0)  # Initial plot

try:
    while True:
        x, y, z = read_serial_data(serial_conn)
        if x is not None and y is not None:
            # For this example, let's assume yaw is fixed at 0 degrees
            update_arrow(ax, x, y, z)
except KeyboardInterrupt:
    print("Program stopped by user.")
finally:
    serial_conn.close()
    plt.ioff()
    plt.show()
