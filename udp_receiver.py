import socket
import struct

RECEIVER_IP = "127.0.0.1"
SENDER_PORT = 54321
MAX_BUFFER_SIZE = 65507
JOINT_DATA_SIZE = struct.calcsize("ii3f")
NUM_JOINTS = 26
NUM_HANDS = 2

def receive_data():
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to a specific address and port
    sock.bind((RECEIVER_IP, SENDER_PORT))

    print(f"Listening on {RECEIVER_IP}:{SENDER_PORT}")

    try:
        while True:
            data, addr = sock.recvfrom(MAX_BUFFER_SIZE)
            print(len(data))

            expected_size = NUM_HANDS * NUM_JOINTS * JOINT_DATA_SIZE

            if len(data) != expected_size:
                print(f"Received data size ({len(data)}) does not match the expected size ({expected_size})")
                continue
   
            for i in range(NUM_JOINTS * NUM_HANDS):
                # Unpack the data using the format string
                joint_data = struct.unpack("ii3f", data[i:i + JOINT_DATA_SIZE])

                hand = joint_data[0]
                joint_index = joint_data[1]
                pose_x, pose_y, pose_z = joint_data[2:5]

                print(f"Hand: {hand}, Joint Index: {joint_index}, Pose: ({pose_x}, {pose_y}, {pose_z})")

            print("End of buffer\n")

    finally:
        sock.close()

if __name__ == "__main__":
    receive_data()
