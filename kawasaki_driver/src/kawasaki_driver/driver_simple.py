#!/usr/bin/env python

import threading
import time
import copy
import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from constants import const
import kawasaki_commands as kc

pose_state_message = None
joint_state_message = None
connection_socket_send = None
connection_socket_receive = None


def set_pose_callback(pose_msg, args):
    rospy.loginfo('Received new target pose.')
    global pose_state_message, joint_state_message
    connection_socket = args[0]
    set_state_lock = args[1]
    get_state_lock = args[2]
    complete_pub = args[3]
    with get_state_lock:
        current_pose = copy.deepcopy(pose_state_message)
    if current_pose is None:
        rospy.logerr('Could not get the current pose! Exiting callback.')
        return
    target_pose = pose_msg

    counter = 0.0
    with set_state_lock:
        if (kc.set_state(pose_msg, connection_socket)):
            if const.wait_until_executed:
                while not kc.compare_two_pose_msgs(current_pose, target_pose):
                    with get_state_lock:
                        current_pose = copy.deepcopy(pose_state_message)
                    time.sleep(const.TIMEOUT_MEDIUM)
                    if counter > const.counter_threshold:
                        rospy.logerr('Could not reach the target pose in ' +
                                     str(const.counter_threshold * const.
                                         TIMEOUT_MEDIUM) + ' sec. Aborting!')
                        kc.clear_buffer(connection_socket)
                        return
                    counter += 1.0
                if (kc.receive_after_motion_complete(connection_socket)):
                    complete_pub.publish(True)
                    rospy.loginfo('Pose reached!')
                else:
                    rospy.logerr(
                        'Received unknown message upon motion completion!')
        else:
            rospy.logerr('Could not move the robot to the new state!')


def shutdown_hook():
    global connection_socket_send, connection_socket_receive
    connection_socket_send.close()
    time.sleep(const.TIMEOUT_SMALL)
    connection_socket_receive.close()
    rospy.loginfo('Shutting down!')


def main():
    global pose_state_message, joint_state_message
    global connection_socket_send, connection_socket_receive
    set_state_lock = threading.Lock()
    get_state_lock = threading.Lock()
    rospy.init_node('kawasaki_driver', disable_signals=True)
    rospy.on_shutdown(shutdown_hook)

    rate = rospy.Rate(const.ROS_RATE)

    connection_socket_send, connection_socket_receive = kc.connect_to_robot(
        const.HOSTNAME, const.PORT)

    pose_pub = rospy.Publisher(
        'pose', PoseStamped, queue_size=const.PUB_QUEUE_SIZE)
    joint_pub = rospy.Publisher(
        'joint_states', JointState, queue_size=const.PUB_QUEUE_SIZE)
    complete_pub = rospy.Publisher(
        'completed_move', Bool, queue_size=const.PUB_QUEUE_SIZE)

    pose_sub = rospy.Subscriber(
        "command/pose",
        PoseStamped,
        set_pose_callback, (connection_socket_send, set_state_lock,
                            get_state_lock, complete_pub),
        queue_size=const.SUB_QUEUE_SIZE)

    while not rospy.is_shutdown():
        with get_state_lock:
            joint_state_message, pose_state_message = kc.get_state(
                connection_socket_receive)
        if joint_state_message is None or pose_state_message is None:
            rate.sleep()
            continue
        pose_pub.publish(pose_state_message)
        joint_pub.publish(joint_state_message)
        rate.sleep()


if __name__ == '__main__':
    main()
