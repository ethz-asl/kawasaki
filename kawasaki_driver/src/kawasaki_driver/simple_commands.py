import socket
import socketserver
import time

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


def get_state():
    global connection_socket
    connection_socket.send(b'wh\n')
    time.sleep(0.1)
    message = connection_socket.recvmsg(500)
    print(message)
    # TODO(ntonci): Parse the message


def set_state():
    global connection_socket
    # TODO(ntonci): Check how can we send the pose in world frame and start
    # motion of the robot to go to that pose
    connection_socket.send('')
    # TODO(ntonci): We need to wait here for the feedback, it should be
    # on-blocking I guess
    message = connection_socket.recvmsg(500)
    print(message)
    # TODO(ntonci): Check the output and parse it to check if the execution was
    # done properly
