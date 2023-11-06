import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Initialize serial connection to the Arduino
arduino = serial.Serial('COM4', 115200, timeout=1)

def compute_orientation(ax, ay, az):
    """
    Compute roll and pitch angles from accelerometer data.
    Assuming ax, ay, az are in units of g.
    """
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

    # Optional: Convert radians to degrees
    # roll = np.degrees(roll)
    # pitch = np.degrees(pitch)

    return roll, pitch
# Constants for the low-pass filter
alpha = 0.5
last_ax, last_ay, last_az = 0, 0, 0
last_gx, last_gy, last_gz = 0, 0, 0

def low_pass_filter(new_data, last_data):
    return alpha * new_data + (1 - alpha) * last_data
def draw_cube(ax, orientation=[0, 0, 0]):
    # Define the vertices of a unit cube
    vertices = np.array([[-0.5, -0.5, -0.5],
                         [0.5, -0.5, -0.5 ],
                         [0.5, 0.5, -0.5],
                         [-0.5, 0.5, -0.5],
                         [-0.5, -0.5, 0.5],
                         [0.5, -0.5, 0.5 ],
                         [0.5, 0.5, 0.5],
                         [-0.5, 0.5, 0.5]])

    # Define the sides of the cube
    faces = [[vertices[j] for j in [0,1,2,3]],
             [vertices[j] for j in [4,5,6,7]], 
             [vertices[j] for j in [0,3,7,4]], 
             [vertices[j] for j in [2,1,5,6]], 
             [vertices[j] for j in [0,1,5,4]],
             [vertices[j] for j in [2,3,7,6]]]

    # Create a 3D axis for plotting
    ax.clear()
    ax.set_xlim([-1, 1]), ax.set_ylim([-1, 1]), ax.set_zlim([-1, 1])

    # Rotate the cube (this part is simplified and assumes rotation is directly given)
    # In a real application, convert gyro data to angles or use accelerometer data directly
    rotation_matrix = np.array([[np.cos(orientation[2]) * np.cos(orientation[1]), np.cos(orientation[2]) * np.sin(orientation[1]) * np.sin(orientation[0]) - np.sin(orientation[2]) * np.cos(orientation[0]), np.cos(orientation[2]) * np.sin(orientation[1]) * np.cos(orientation[0]) + np.sin(orientation[2]) * np.sin(orientation[0])],
                                [np.sin(orientation[2]) * np.cos(orientation[1]), np.sin(orientation[2]) * np.sin(orientation[1]) * np.sin(orientation[0]) + np.cos(orientation[2]) * np.cos(orientation[0]), np.sin(orientation[2]) * np.sin(orientation[1]) * np.cos(orientation[0]) - np.cos(orientation[2]) * np.sin(orientation[0])],
                                [-np.sin(orientation[1]), np.cos(orientation[1]) * np.sin(orientation[0]), np.cos(orientation[1]) * np.cos(orientation[0])]])

    # Apply the rotation to each face
    faces_rotated = np.dot(faces, rotation_matrix)
    
    # Create a Poly3DCollection object from the faces
    poly3d = [[tuple(vertex) for vertex in face] for face in faces_rotated]
    ax.add_collection3d(Poly3DCollection(poly3d, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

    plt.pause(0.001)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Main loop to read data from the serial port and update the cube orientation
while True:
    line = arduino.readline().decode('utf-8').strip()
    if line.startswith('a/g:'):
        try:
            parts = line.split()
            ax_val, ay_val, az_val, gx_val, gy_val, gz_val = map(float, parts[1:7])

            # Convert accelerometer values to g's if they're not already
            ax_val /= 16384.0  # Assuming a full scale range of +/-2g for the MPU6050
            ay_val /= 16384.0
            az_val /= 16384.0

            # Apply the low-pass filter to the accelerometer and gyroscope values
            ax_val = low_pass_filter(ax_val, last_ax)
            ay_val = low_pass_filter(ay_val, last_ay)
            az_val = low_pass_filter(az_val, last_az)
            # Gyro values could also be filtered, but it depends on how they will be used.

            # Compute roll and pitch from the filtered accelerometer values
            roll, pitch = compute_orientation(ax_val, ay_val, az_val)

            # Update last values
            last_ax, last_ay, last_az = ax_val, ay_val, az_val

            # Update the cube orientation
            # The gyroscope values (gx_val, gy_val, gz_val) can be integrated over time to compute the yaw,
            # but without a magnetometer, yaw will drift over time.
            draw_cube(ax, [roll, pitch, 0])  # Assuming yaw is 0 for simplicity
        except ValueError as e:
            print(f"Value error with line: {line}, error: {e}")

arduino.close()
