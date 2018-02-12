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
    ALREADY_RUNNING_ERROR = ('(P1009)Program is already running.')
    CANNOT_REACH_ERROR = ('(P1013)Cannot execute because in error now. '
                          'Reset error.')
    NO_ERROR_FEEDBACK = ('>')
    MOTION_COMPLETE_MESSAGE = ('DO motion completed.\r\n')
    MIN_STATE_MSG_SIZE = 230  # Probably a bit higher, conservative estimate
    STATE_MSG_PARSED_SIZE = 5
    STATE_MSG_JOINT_VALUES_INDEX = 2
    STATE_MSG_POSE_VALUES_INDEX = 4

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
    TRANSLATION_DIFFERENCE_THRESHOLD = 0.0001
    ROTATION_DIFFERENCE_THRESHOLD = 0.05

    # PARAMS:
    wait_until_executed = True
    counter_threshold = 40  # x0.5 sec wait

    # TOPICS:
    pose_pub_topic = 'pose'
    joint_pub_topic = 'joint_states'
    completed_move_pub_topic = 'completed_move'
    pose_sub_topic = 'command/pose'
