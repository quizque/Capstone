import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Configure serial port
serial_port = "COM11"  # Adjust based on your system
baud_rate = 115200  # Adjust based on your device

# Open serial port
ser = serial.Serial(serial_port, baud_rate, timeout=1)
time.sleep(2)  # Allow time for the connection to be established

# Set up the figure for plotting
plt.ion()  # Enable interactive mode
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Initial plot setup
(line,) = ax.plot([0, 0], [0, 0], [0, 0], "-o")
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")


def update_line(yaw, pitch, roll):
    # Convert yaw, pitch, roll to radians
    yaw = np.deg2rad(yaw)
    pitch = np.deg2rad(pitch)
    roll = np.deg2rad(roll)

    # Initial direction vector along x-axis
    direction = np.array([1, 0, 0])

    # Rotation matrices
    Rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )

    Ry = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    Rx = np.array(
        [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]]
    )

    # Apply rotations
    direction = Rz @ Ry @ Rx @ direction

    # Update plot data
    line.set_data([0, direction[0]], [0, direction[1]])
    line.set_3d_properties([0, direction[2]])
    plt.draw()
    plt.pause(1 / 60)


# Main loop to read data and update plot
while True:
    try:
        # Read a line of data from the serial port
        ser.flush()
        ser.flushInput()
        ser.flushOutput()
        data = ser.readline().decode("utf-8").strip()

        print(data)

        # Extract yaw, pitch, roll
        if data.startswith("Theta:"):
            try:
                parts = data.split()
                yaw = float(parts[1])
                pitch = float(parts[2])
                roll = float(parts[3])

                update_line(yaw, pitch, roll)
            except ValueError as e:
                print("Error parsing data:", e)

    except KeyboardInterrupt:
        print("Exiting...")
        break

# Close serial port when done
ser.close()
