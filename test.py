import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Initialize serial connection to the Arduino
arduino = serial.Serial('COM4', 115200, timeout=1)

# Replace 'COM_PORT' with your actual Arduino's COM port number

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
            # Use the split() function with no arguments, which will split on any whitespace
            parts = line.split()
            # Now the parts list should have the values as its elements 1 through 6
            ax_val, ay_val, az_val, gx_val, gy_val, gz_val = map(int, parts[1:7])
            
            # Compute angles or orientation from ax, ay, az, gx, gy, gz as required

            # Example: Update the visualization with dummy orientation values
            # This should be replaced with actual angle calculations
            draw_cube(ax, [np.deg2rad(ax_val), np.deg2rad(ay_val), np.deg2rad(gx_val)])
        except ValueError as e:
            # Handle the case where a line is incomplete or corrupted
            print(f"Value error with line: {line}, error: {e}")


arduino.close()
