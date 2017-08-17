import socket
import time
import math
import threading

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf
import rospy


def establish_socket_connection(socekt_connection, address_port,
                                connection_name, socket_number):
    try:
        socekt_connection = socket.create_connection(address_port, timeout=5)
        print("Sending socket established.")
        time.sleep(0.1)
    except socket.timeout:
        print("Sending socket connection could not be established.")
        exit()
    message = socekt_connection.recvfrom(500)  # in python3: recvfrom
    if (message ==
        ('\xff\xfd\x18\xff\xfb\x01\xff\xfa\x18\x01\x01\xff\xf0\xff\xfb\x03\rConnecting to Kawasaki E Controller\r\n\r\nlogin: ',
         None)):
        socekt_connection.send(b'as\n')
        time.sleep(1)
        print("Logging in as 'as'")
    else:
        print("Got wrong login request information, exiting!")
        exit()
    message = socekt_connection.recvfrom(500)
    if (message == ('as\nThis is AS monitor terminal "AUX' + str(socket_number)
                    + '"\r\n>', None)):
        print("Logged in successfully, for " + connection_name)
    else:
        print("Didn't log in, but got the following message:")
        print(message)
        socekt_connection.close()
        exit()
    return socekt_connection


def connect_to_robot(address="192.168.2.2", port=23):
    """Open two socket connections and login in as "as".

    The first socket is for sending commands and the second is for receiving.
    """
    connection_socket_send = None
    connection_socket_receive = None
    connection_socket_send = establish_socket_connection(
        connection_socket_send, (address, port), "send", 1)
    time.sleep(1)
    connection_socket_receive = establish_socket_connection(
        connection_socket_receive, (address, port), "receive", 2)

    # TODO(ntonci): Check message and write a parser to check if proper
    # connection was established
    # TODO(ntonci): Check if there is a nicer way to read msgs since there
    # might be some delay in-between commands
    return connection_socket_send, connection_socket_receive


def state_message_parser(message):
    split_msg = message.split('\r\n')
    joint_values = parse_and_convert_to_float(split_msg[2])
    pose_values = parse_and_convert_to_float(split_msg[4])

    ros_now = rospy.Time.now()

    joint_values = [math.radians(angle) for angle in joint_values]

    joint_state_message = JointState()
    joint_state_message.header = Header()
    joint_state_message.header.stamp = ros_now
    joint_state_message.header.frame_id = 'joint'
    joint_state_message.name = split_msg[1].split()
    joint_state_message.position = joint_values
    joint_state_message.velocity = []
    joint_state_message.effort = []

    # pose_state_message = JointState()
    # pose_state_message.header = Header()
    # pose_state_message.header.stamp = ros_now
    # pose_state_message.name = split_msg[3].split()
    # pose_state_message.position = pose_values
    # pose_state_message.velocity = []
    # pose_state_message.effort = []

    pose_state_message = PoseStamped()
    pose_state_message.header = Header()
    pose_state_message.header.stamp = ros_now
    pose_state_message.header.frame_id = 'base'
    pose_state_message.pose.position.x = pose_values[0] / 1000.
    pose_state_message.pose.position.y = pose_values[1] / 1000.
    pose_state_message.pose.position.z = pose_values[2] / 1000.
    quaternion = tf.transformations.quaternion_from_euler(
        math.radians(pose_values[3]),
        math.radians(pose_values[4]), math.radians(pose_values[5]), 'szyz')
    pose_state_message.pose.orientation.x = quaternion[0]
    pose_state_message.pose.orientation.y = quaternion[1]
    pose_state_message.pose.orientation.z = quaternion[2]
    pose_state_message.pose.orientation.w = quaternion[3]

    return joint_state_message, pose_state_message


def get_state(connection_socket):
    connection_socket.send(b'wh\n')
    time.sleep(0.08)
    message = connection_socket.recvfrom(500)
    if (message[0][0:66] ==
            'wh\r\n     JT1       JT2       JT3       JT4       JT5       JT6  \r\n'):
        joint_states_unparsed = message[0]
        joint_state_message, pose_state_message = state_message_parser(
            joint_states_unparsed)
        return joint_state_message, pose_state_message
    else:
        print("Got wrong return message after executing 'wh':")
        print(message)
    # TODO(ntonci): Parse the message


def set_state(pose_msg, args):
    print("Got pose_msg")
    connection_socket = args[0]
    set_state_lock = args[1]
    with set_state_lock:
        command_string = create_lmove_message(pose_msg)
        connection_socket.send(command_string)

        message = connection_socket.recvfrom(500)
        print(message)
    # TODO(ntonci): Check the output and parse it to check if the execution was
    # done properly


def create_lmove_message(pose):
    quaternion = (pose.pose.orientation.x, pose.pose.orientation.y,
                  pose.pose.orientation.z, pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion, 'szyz')
    euler = [math.degrees(angle) for angle in euler]
    translation = (pose.pose.position.x, pose.pose.position.y,
                   pose.pose.position.z)
    message = b'DO LMOVE TRANS('
    for pos in translation:
        message = message + str(round(pos * 1000., 3)) + ', '
    for rot in euler:
        message = message + str(round(rot, 3)) + ', '
    message = message[:-2] + ')\n'
    return message


def parse_and_convert_to_float(string_message):
    split_string = string_message.split()
    float_value = []
    for value in split_string:
        float_value.append(float(value))
    return float_value


def read_from_socket(connection_socket):
    message = connection_socket.recvfrom(500)
    # Check if the type of message is joint states.
    if (message[0][0:71] ==
            b'wh\r\n     JT1       JT2       JT3       JT4       JT5       JT6  \r\n'):
        print("Got joint_state message.")
        joint_states_unparsed = message[0]
        # TODO(ff): trigger joint_state parsing thread.
