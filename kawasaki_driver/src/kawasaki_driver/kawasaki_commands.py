import socket
import time
import sys

import rospy

import parsers
from constants import const


def establish_socket_connection(socket_connection, address_port,
                                connection_name, socket_number):
    """ Opens the socket connection on the specified address and port.
    """
    try:
        socket_connection = socket.create_connection(
            address_port, timeout=const.SOCKET_TIMEOUT)
        rospy.loginfo(connection_name + " socket established.")
    except socket.timeout:
        rospy.logerr(connection_name +
                     " socket connection could not be established.")
        sys.exit()
    time.sleep(const.TIMEOUT_MEDIUM)
    message = socket_connection.recvfrom(const.RECEIVE_MSG_SIZE)
    if (message == const.CONNECTION_STRING_TUPLE):
        if (socket_connection.send(const.LOGIN_USER) is
                not len(const.LOGIN_USER)):
            rospy.logerr(
                "Proper command was not sent to the robot. Could not login.")
            sys.exit()
        rospy.loginfo("Logging in as 'as'")
    else:
        rospy.logerr("Got wrong login request information, exiting!")
        rospy.logerr(message)
        socket_connection.close()
        sys.exit()
    time.sleep(const.TIMEOUT_LARGE)
    message = socket_connection.recvfrom(const.RECEIVE_MSG_SIZE)
    if (message == parsers.login_string_tuple_creator(socket_number)):
        rospy.loginfo("Logged in successfully, for " + connection_name)
    else:
        rospy.logerr("Did not log in. Rececived the following message:")
        rospy.logerr(message)
        socket_connection.close()
        sys.exit()
    return socket_connection


def connect_to_robot(address="192.168.2.2", port=23):
    """ Open two socket connections and login in as "as".
    The first socket is for sending commands and the second is for receiving.
    """
    connection_socket_send = None
    connection_socket_receive = None
    connection_socket_send = establish_socket_connection(
        connection_socket_send, (address, port), "send", 1)
    time.sleep(const.TIMEOUT_LARGE)
    connection_socket_receive = establish_socket_connection(
        connection_socket_receive, (address, port), "receive", 2)
    return connection_socket_send, connection_socket_receive


def get_state(connection_socket):
    """ Get the current joint state and pose of the robot.
    """
    if (connection_socket.send(const.GET_STATE) is not len(const.GET_STATE)):
        rospy.logerr("Proper command was not sent to the robot.")
        return None, None
    time.sleep(const.TIMEOUT_MEDIUM)
    message = connection_socket.recvfrom(const.RECEIVE_MSG_SIZE)
    if (message[0][0:66] == const.STATE_STRING_PARTIAL
            and len(message[0]) > const.MIN_STATE_MSG_SIZE):
        joint_states_unparsed = message[0]
        joint_state_message, pose_state_message = parsers.state_message_parser(
            joint_states_unparsed)
        return joint_state_message, pose_state_message
    else:
        rospy.logerr("Got wrong return message after executing 'wh':")
        rospy.logerr(message)
        return None, None


def set_state(pose_msg, connection_socket):
    """ Set the new target pose for the robot.
    """
    command_string = parsers.create_lmove_message(pose_msg)
    if (connection_socket.send(command_string) is not len(command_string)):
        rospy.logerr("Proper command was not sent to the robot.")
    time.sleep(const.TIMEOUT_MEDIUM)
    message = connection_socket.recvfrom(const.RECEIVE_MSG_SIZE)
    feedback_command, feedback_error = parsers.move_feedback_msg_parser(
        message)
    if ((feedback_command + b'\n') == command_string):
        if (feedback_error == const.NO_ERROR_FEEDBACK):
            return True
        elif (feedback_error == const.MOTOR_OFF_ERROR):
            rospy.logerr(feedback_error)
            return False
        elif (feedback_error == const.TEACH_LOCK_ON_ERROR):
            rospy.logerr(feedback_error)
            return False
        elif (feedback_error == const.ERROR_MODE_ERROR):
            rospy.logerr(feedback_error)
            return False
        elif (feedback_error == const.ALREADY_RUNNING_ERROR):
            rospy.logerr(feedback_error)
            return False
        elif (feedback_error == const.CANNOT_REACH_ERROR):
            rospy.logerr(feedback_error)
            return False
        else:
            # TODO(ntonci): Handle any other known errors.
            rospy.logerr(message)
            return False
    else:
        rospy.logerr('Unknown error: ')
        rospy.logerr(message)
        return False


def receive_after_motion_complete(connection_socket):
    time.sleep(const.TIMEOUT_LARGE)
    message = connection_socket.recvfrom(const.RECEIVE_MSG_SIZE)
    if (message[0] == const.MOTION_COMPLETE_MESSAGE):
        return True
    else:
        rospy.logerr('Received unknown error message:')
        rospy.loginfo(message)
        return False


def clear_buffer(connection_socket):
    rospy.loginfo('Clearing buffer')
    time.sleep(const.TIMEOUT_MEDIUM)
    message = connection_socket.recvfrom(const.RECEIVE_MSG_SIZE)


def compare_two_pose_msgs(current_pose, target_pose):
    """ Checks if the current robot pose and its target pose are the same.
    """
    current_translation, current_euler = parsers.convert_pose_msg_to_translation_euler(
        current_pose)
    target_translation, target_euler = parsers.convert_pose_msg_to_translation_euler(
        target_pose)
    diff_t = 0.0
    diff_r = 0.0
    for i, j in zip(current_translation, target_translation):
        diff_t += pow(i - j, 2.0)
    for i, j in zip(current_euler, target_euler):
        diff_r += pow(i - j, 2.0)
    if ((pow(diff_t, 0.5) < const.translation_difference_threshold)
            and (pow(diff_r, 0.5) < const.rotation_difference_threshold)):
        return True
    else:
        return False
