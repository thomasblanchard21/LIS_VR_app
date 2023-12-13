import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

RECEIVER_IP = "127.0.0.1"
SENDER_PORT = 54321
MAX_BUFFER_SIZE = 65507
JOINT_DATA_SIZE = struct.calcsize("ii7f")
NUM_JOINTS = 26
NUM_HANDS = 2
DISPLAY_COUNT = 3  # Display once every 3 batches


def visualize_joints(ax, joint_data):
    # Clear the previous plot
    ax.cla()

    joint_data = joint_data[(0,1,3,5,7,8,10,12,13,15,17,18,20,22,23,25, 26,27,29,31,33,34,36,38,39,41,43,44,46,48,50,51), :] # Remove some joints

    # Extract x, y, and z coordinates
    x_coords = joint_data['pos_x']
    y_coords = joint_data['pos_y']
    z_coords = joint_data['pos_z']

    # Plot the joints
    ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')

    # Label axes
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    # Invert axes
    ax.invert_xaxis()
    ax.invert_yaxis()
    ax.invert_zaxis()
    
    # Set axis limits
    ax.set_xlim([min(x_coords), max(x_coords)])
    ax.set_ylim([min(y_coords), max(y_coords)])
    ax.set_zlim([min(z_coords), max(z_coords)])

def compute_grasp(joint_data):
    # Compute the grasp
    grasp_left = 0.0
    grasp_right = 0.0

    for i in range(1,6):
        for j in range (i+1,6):
            grasp_left += compute_distance(joint_data[5*i], joint_data[5*j]) / 10 # compute distance between tips of fingers i and j
            grasp_right += compute_distance(joint_data[5*i+NUM_JOINTS], joint_data[5*j+NUM_JOINTS]) / 10

    return grasp_left, grasp_right

def compute_distance(joint1, joint2):
    # Compute the distance between two joints
    return np.sqrt((joint1['pos_x'] - joint2['pos_x'])**2 + (joint1['pos_y'] - joint2['pos_y'])**2 + (joint1['pos_z'] - joint2['pos_z'])**2)


## Relative position of fingertips to the palm, not used

# def quaternion_to_rotation_matrix(quaternion):
#     # Assuming quaternion is in the order (x, y, z, w)
#     q = np.array(quaternion)
#     q_norm = q / np.linalg.norm(q)
#     x, y, z, w = q_norm

#     rotation_matrix = np.array([
#         [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
#         [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
#         [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
#     ])

#     return rotation_matrix

# def compute_relative_position(palm_position, tip_position, finger_orientation):
#     # Convert quaternion to a rotation matrix
#     rotation_matrix = quaternion_to_rotation_matrix(finger_orientation)

#     # Compute the relative position using the formula
#     relative_position = np.dot(rotation_matrix, (tip_position - palm_position))

#     return relative_position


if __name__ == "__main__":

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to a specific address and port
    sock.bind((RECEIVER_IP, SENDER_PORT))

    print(f"Listening on {RECEIVER_IP}:{SENDER_PORT}")

    # Define the dtype for the structured array
    hand_data = np.dtype([('hand', np.int32), ('joint_index', np.int32), ('ori_x', np.float32), ('ori_y', np.float32), ('ori_z', np.float32), ('ori_w', np.float32), ('pos_x', np.float32), ('pos_y', np.float32), ('pos_z', np.float32)])

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    try:
        batch_counter = 0

        while True:
            data, addr = sock.recvfrom(MAX_BUFFER_SIZE)

            expected_size = NUM_HANDS * NUM_JOINTS * JOINT_DATA_SIZE

            if len(data) != expected_size:
                print(f"Received data size ({len(data)}) does not match the expected size ({expected_size})")
                continue

            # Unpack the data using the format string and reshape it into a structured array
            joint_data = np.frombuffer(data, dtype=hand_data).reshape((NUM_JOINTS * NUM_HANDS, 1))

            grasp_left, grasp_right = compute_grasp(joint_data)
            # print(f"Grasp left: {grasp_left[0]}")
            # print(f"Grasp right: {grasp_right[0]}")

            batch_counter += 1

            if batch_counter == DISPLAY_COUNT:
                visualize_joints(ax, joint_data)
                plt.draw()
                plt.pause(0.02)
                batch_counter = 0  # Reset the counter

            # Keeping only 3D position of palm and fingertips, orientation of palm and grasp for each hand
            output_data = []

            # Left hand
            for i in (0,5,10,15,20,25):
                output_data.append(joint_data[i][0]['pos_x'])
                output_data.append(joint_data[i][0]['pos_y'])
                output_data.append(joint_data[i][0]['pos_z'])
            output_data.append(joint_data[0][0]['ori_x'])
            output_data.append(joint_data[0][0]['ori_y'])
            output_data.append(joint_data[0][0]['ori_z'])
            output_data.append(grasp_left[0])

            # Right hand
            for i in (26,31,36,41,46,51):
                output_data.append(joint_data[i][0]['pos_x'])
                output_data.append(joint_data[i][0]['pos_y'])
                output_data.append(joint_data[i][0]['pos_z'])
            output_data.append(joint_data[26][0]['ori_x'])
            output_data.append(joint_data[26][0]['ori_y'])
            output_data.append(joint_data[26][0]['ori_z'])
            output_data.append(grasp_right[0])

    finally:
        sock.close()
