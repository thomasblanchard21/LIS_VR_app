import socket
import struct
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

RECEIVER_IP = "127.0.0.1"
SENDER_PORT = 54321
MAX_BUFFER_SIZE = 65507
FORMAT_STRING = "ii7f"
JOINT_DATA_SIZE = struct.calcsize(FORMAT_STRING)
NUM_JOINTS = 26
NUM_HANDS = 2
DISPLAY_COUNT = 3  # Display once every 3 batches



def compute_grasp(joint_data):
    # Compute the grasp
    grasp_left, grasp_right = 0.0, 0.0

    for i in range(1,6):
        for j in range (i+1,6):
            grasp_left += compute_distance(joint_data[5*i], joint_data[5*j]) / 10 # compute distance between tips of fingers i and j
            grasp_right += compute_distance(joint_data[5*i+NUM_JOINTS], joint_data[5*j+NUM_JOINTS]) / 10

    return grasp_left[0], grasp_right[0]

def compute_distance(joint1, joint2):
    # Compute the distance between two joints
    return np.sqrt((joint1['pos_x'] - joint2['pos_x'])**2 + (joint1['pos_y'] - joint2['pos_y'])**2 + (joint1['pos_z'] - joint2['pos_z'])**2)

# Relative position of fingertips to the palm
def quaternion_to_rotation_matrix(quaternion):
    # Assuming quaternion is in the order (x, y, z, w)
    q = np.array(quaternion)
    q_norm = q / np.linalg.norm(q)
    x, y, z, w = q_norm

    rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])

    return rotation_matrix

def quaternion_to_euler(quaternion):
    # Assuming quaternion is in the order (x, y, z, w)
    x, y, z, w = quaternion

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x**2 + y**2)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y**2 + z**2)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

def compute_relative_position(palm_position, tip_position, finger_orientation):

    # Check for NaN values
    if(np.isnan(palm_position).any() or np.isnan(tip_position).any() or np.isnan(finger_orientation).any()):
        return np.array([np.nan, np.nan, np.nan])
    
    # Return absolute position for palm
    elif (palm_position == tip_position).all():
        return palm_position
    
    else:
        # Convert quaternion to a rotation matrix
        rotation_matrix = quaternion_to_rotation_matrix(finger_orientation)

        # Compute the relative position using the formula
        relative_position = np.dot(rotation_matrix, (tip_position - palm_position))

        return relative_position


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

        while True:
            data, addr = sock.recvfrom(MAX_BUFFER_SIZE)

            expected_size = NUM_HANDS * NUM_JOINTS * JOINT_DATA_SIZE + struct.calcsize('d')

            if len(data) != expected_size:
                print(f"Received data size ({len(data)}) does not match the expected size ({expected_size})")
                continue

            # Unpack the simulation time
            sim_time = struct.unpack('d', data[:struct.calcsize('d')])[0]
            # print(f"Simulation time: {sim_time}")

            # Unpack the data using the format string and reshape it into a structured array
            joint_data = np.frombuffer(data[struct.calcsize('d'):], dtype=hand_data).reshape((NUM_JOINTS * NUM_HANDS, 1))

            # Compute the grasp
            grasp_left, grasp_right = compute_grasp(joint_data)
            if grasp_left == 0.0:
                grasp_left = np.nan
            if grasp_right == 0.0:
                grasp_right = np.nan

            # print(f"Grasp left: {grasp_left}")
            # print(f"Grasp right: {grasp_right}")

            # Convert the structured array into a Pandas DataFrame
            joint_data = pd.DataFrame(joint_data.flatten(), columns=['hand', 'joint_index', 'ori_x', 'ori_y', 'ori_z', 'ori_w', 'pos_x', 'pos_y', 'pos_z'])
            joint_data.replace(100, np.nan, inplace=True)   # Replace 100 with NaN

            # Keep only palms and fingertips
            joint_data = joint_data[joint_data['joint_index'].isin([0,5,10,15,20,25])]

            # Compute relative position of fingertips to the palm
            joint_data[['pos_x', 'pos_y', 'pos_z']] = pd.DataFrame(joint_data.apply(lambda x: compute_relative_position(
                joint_data.loc[x['hand'] * NUM_JOINTS]['pos_x':'pos_z'].values,
                x['pos_x':'pos_z'].values,
                x['ori_x':'ori_w'].values
            ), axis=1).to_list(), columns=['pos_x', 'pos_y', 'pos_z'], index=joint_data.index)

            # ----------------------------------------------------------------------------------------------
            #
            # OUTPUT DATA:
            #
            # Output data is a list with the following elements:
            #
            #   - sim_time: simulation time (double)
            #
            #   - joint_data: Pandas DataFrame with the following columns:
            #       - hand: 0 for left hand, 1 for right hand
            #       - joint_index: 0 for palm, 5 for thumb, 10 for index finger, 15 for middle finger, 20 for ring finger, 25 for pinky
            #       - ori_x, ori_y, ori_z, ori_w: orientation of the joint in quaternion form
            #       - pos_x, pos_y, pos_z: relative position of the joint to the palm, absolute position for the palm
            #
            #   - grasp_left: grasp value for the left hand (float)
            #   - grasp_right: grasp value for the right hand (float)
            #
            # ---------------------------------------------------------------------------------------------

            output_data = []
            output_data.append(sim_time)
            output_data.append(joint_data)
            output_data.append(grasp_left)
            output_data.append(grasp_right)
            
            print(output_data)

    finally:
        sock.close()
