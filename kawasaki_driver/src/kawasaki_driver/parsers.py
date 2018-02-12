import math
import time

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from constants import const


def state_message_parser(message):
    split_msg = message.split('\r\n')
    if (len(split_msg) < const.STATE_MSG_PARSED_SIZE):
        return None, None
    joint_values = parse_and_convert_to_float(
        split_msg[const.STATE_MSG_JOINT_VALUES_INDEX])
    pose_values = parse_and_convert_to_float(
        split_msg[const.STATE_MSG_POSE_VALUES_INDEX])

    ros_now = rospy.Time()
    try:
        ros_now = rospy.Time.now()
    except rospy.ROSInitException:
        current_system_time = time.time()
        ros_now.secs = int(current_system_time)
        ros_now.nsecs = int(
            (current_system_time - int(current_system_time)) * 1e9)

    joint_values = [math.radians(angle) for angle in joint_values]

    try:
        joint_state_message = JointState()
        joint_state_message.header = Header()
        joint_state_message.header.stamp = ros_now
        joint_state_message.header.frame_id = 'joint'
        joint_state_message.name = split_msg[1].split()
        joint_state_message.position = joint_values
        joint_state_message.velocity = []
        joint_state_message.effort = []

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
    except IndexError, e:
        rospy.logerr(e)
        return None, None

    return joint_state_message, pose_state_message


def move_feedback_msg_parser(message):
    split_msg = message[0].split('\r\n')
    feedback_command = split_msg[0]
    feedback_error = split_msg[1]
    return feedback_command, feedback_error


def parse_and_convert_to_float(string_message):
    split_string = string_message.split()
    float_value = []
    for value in split_string:
        float_value.append(float(value))
    return float_value


def create_lmove_message(pose_msg):
    translation, euler = convert_pose_msg_to_translation_euler(pose_msg)
    message = b'DO LMOVE TRANS('
    for pos in translation:
        message = message + str(round(pos * 1000., 3)) + ', '
    for rot in euler:
        message = message + str(round(rot, 3)) + ', '
    message = message[:-2] + ')\n'
    return message


def convert_pose_msg_to_translation_euler(pose_msg):
    translation = (pose_msg.pose.position.x, pose_msg.pose.position.y,
                   pose_msg.pose.position.z)
    quaternion = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                  pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion, 'szyz')
    euler = [math.degrees(angle) for angle in euler]
    return translation, euler


def login_string_tuple_creator(socket_number):
    return (
        'as\nThis is AS monitor terminal "AUX' + str(socket_number) + '"\r\n>',
        None)
