import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


# Function to convert quaternion to rotation matrix
def quaternion_to_rotation_matrix(q):
    q0, q1, q2, q3 = q
    R = np.array(
        [
            [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1**2 + q2**2)],
        ]
    )
    return R


# Function to plot the heading
def plot_heading(R, ax):
    ax.clear()

    # Plot the axes
    ax.quiver(0, 0, 0, R[0, 0], R[1, 0], R[2, 0], color="r", length=1)
    ax.quiver(0, 0, 0, R[0, 1], R[1, 1], R[2, 1], color="g", length=1)
    ax.quiver(0, 0, 0, R[0, 2], R[1, 2], R[2, 2], color="b", length=1)

    # Set the labels and limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.draw()
    plt.pause(0.01)  # Pause to update the plot


# Serial port configuration
serial_port = "COM10"  # Update this with your serial port
baud_rate = 115200

# Open the serial port
ser = serial.Serial(serial_port, baud_rate)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

try:
    while True:
        # Read a line from the serial port
        ser.reset_input_buffer()
        line = ser.readline().decode("utf-8").strip()
        print(f"Received: {line}")

        # Parse the quaternion
        q = list(map(float, line.split(",")))

        # Convert the quaternion to a rotation matrix
        R = quaternion_to_rotation_matrix(q)

        # Plot the heading
        plot_heading(R, ax)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    ser.close()
    plt.close(fig)
