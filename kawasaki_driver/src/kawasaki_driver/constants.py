class const:
    # MESSAGES:
    CONNECTION_STRING_TUPLE = (('\xff\xfd\x18\xff\xfb\x01\xff\xfa\x18\x01'
                                '\x01\xff\xf0\xff\xfb\x03\rConnecting to '
                                'Kawasaki E Controller\r\n\r\nlogin: '), None)
    STATE_STRING_PARTIAL = ('wh\r\n     JT1       JT2       JT3'
                            '       JT4       JT5       JT6  \r\n')
    MOTOR_OFF_ERROR = ('(P1000)Cannot execute program because motor power'
                       ' is OFF.')
    TEACH_LOCK_ON_ERROR = ('(P1002)Cannot execute program because teach lock '
                           'is ON.')
    ERROR_MODE_ERROR = ('(P1013)Cannot execute because in error now. '
                        'Reset error.')
    MOTION_COMPLETE_MESSAGE = ('DO motion completed.\r\n')
    MIN_STATE_MSG_SIZE = 230  # Probably a bit higher, conservative estimate

    LOGIN_USER = b'as\n'
    GET_STATE = b'wh\n'

    # SETUP:
    PORT = 23  # 10 Hz, RobotState
    HOSTNAME = "192.168.2.2"
    RECEIVE_MSG_SIZE = 1500
    SOCKET_TIMEOUT = 5
    ROS_RATE = 6  # ~6 Hz main loop
    PUB_QUEUE_SIZE = 10
    SUB_QUEUE_SIZE = 1
    TIMEOUT_SMALL = 0.1
    TIMEOUT_MEDIUM = 0.5
    TIMEOUT_LARGE = 1.0

    # PARAMS:
    wait_until_executed = True
    counter_threshold = 10
