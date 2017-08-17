import socket
import socketserver
import time
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf
import rospy

connection_socket


def connect_to_robot(address="192.168.2.2", port=23):
    global connection_socket
    connection_socket = socket.create_connection(address, port)
    connection_socket.recvmsg(500)
    connection_socket.send(b'as\n')
    message = connection_socket.recvmsg(500)
    print(message)
    # TODO(ntonci): Check message and write a parser to check if proper
    # connection was established
    # TODO(ntonci): Check if there is a nicer way to read msgs since there
    # might be some delay in-between commands


def state_message_parser(message):
    split_msg = message.split('\r\n')
    joint_values = parse_and_convert_to_float(split_msg[2])
    pose_values = parse_and_convert_to_float(split_msg[4])

    joint_state_message = JointState()
    joint_state_message.header = Header()
    joint_state_message.header.stamp = rospy.Time.now()
    joint_state_message.name = split_msg[1].split()
    joint_state_message.position = joint_values
    joint_state_message.velocity = []
    joint_state_message.effort = []

    pose_state_message = JointState()
    pose_state_message.header = Header()
    pose_state_message.header.stamp = rospy.Time.now()
    pose_state_message.name = split_msg[3].split()
    pose_state_message.position = pose_values
    pose_state_message.velocity = []
    pose_state_message.effort = []

    return joint_state_message, pose_state_message


def create_pose_message(pose):
    quaternion = (pose.pose.orientation.x, pose.pose.orientation.y,
                  pose.pose.orientation.z, pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(
        quaternion)  # TODO(ntonci): Check order of angles and units
    euler = [math.degrees(angle) for angle in euler]
    translation = (pose.pose.position.x, pose.pose.position.y,
                   pose.pose.position.z)
    message = 'LMOVE '
    for pos in translation:
        message = message + str(pos) + ', '
    for rot in euler:
        message = message + str(rot) + ', '
    message = message[:-2] + '\n'
    return message


def parse_and_convert_to_float(string_message):
    split_string = string_message.split()
    float_value = []
    for value in split_string:
        float_value.append(float(value))
    return float_value
