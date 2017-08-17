import socket
# import socketserver
import time


def establish_socket_connection(socekt_connection, address_port,
                                connection_name, socket_number):
    try:
        socekt_connection = socket.create_connection(address_port,
                                                          timeout=5)
        print("Sending socket established.")
        time.sleep(0.1)
    except socket.timeout:
        print("Sending socket connection could not be established.")
        exit()
    message = socekt_connection.recvfrom(500)  # in python3: recvmsg
    if (message == ('\xff\xfd\x18\xff\xfb\x01\xff\xfa\x18\x01\x01\xff\xf0\xff\xfb\x03\rConnecting to Kawasaki E Controller\r\n\r\nlogin: ', None)):
        socekt_connection.send(b'as\n')
        time.sleep(1)
        print("Logging in as 'as'")
    else:
        print("Got wrong login request information, exiting!")
        exit()
    message = socekt_connection.recvfrom(500)
    if (message == ('as\nThis is AS monitor terminal "AUX' +
                    str(socket_number) + '"\r\n>', None)):
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

    connection_socket_send.close()
    time.sleep(1)
    connection_socket_receive.close()
    # TODO(ntonci): Check message and write a parser to check if proper
    # connection was established
    # TODO(ntonci): Check if there is a nicer way to read msgs since there
    # might be some delay in-between commands
    exit()

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

def read_from_socket(connection_socket):
    message = connection_socket.recvmsg(500)
    # Check if the type of message is joint states.
    if (message[0][0:71] == b'wh\r\n     JT1       JT2       JT3       JT4       JT5       JT6  \r\n'):
        print("Got joint_state message.")
        joint_states_unparsed = message[0]
        # TODO(ff): trigger joint_state parsing thread.
