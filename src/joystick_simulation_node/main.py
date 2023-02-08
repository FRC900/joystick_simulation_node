#!/usr/bin/env python3

import tf2_ros
import rospy
from threading import Thread
import os
import signal

from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode

import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from joystick_simulation_node.joystick import Joystick as UIJoystick
from ck_ros_base_msgs_node.msg import *
from enum import Enum

class State(Enum):
    AUTO = 1
    TELEOP = 2
    TEST = 3

show_drive = True
show_xbox = True
show_button_box = True

left_stick = None
right_stick = None
xbox_left_stick = None
xbox_right_stick = None
button_box_buttons = []
xbox_button_box_buttons = []
driver_buttons = []
app = None
widget = None
robot_state = State.AUTO
robot_enabled = False
left_twist = 0
right_twist = 0
xbox_left_trigger = 0
xbox_right_trigger = 0

class MainWindow(QMainWindow):
    def __init__(self):
        global show_drive
        global show_xbox
        global show_button_box
        global left_stick
        global right_stick
        global xbox_left_stick
        global xbox_right_stick

        super().__init__()

        self.setWindowTitle('Joystick Simulator')

        cw = QWidget()
        ml = QGridLayout()
        cw.setLayout(ml)
        self.setCentralWidget(cw)

        enable_disable_button = QPushButton("Enable")
        enable_disable_button.clicked.connect(lambda checked, button=enable_disable_button: toggle_enable_disable(button))

        radio_box = QWidget()
        radio_box_layout = QGridLayout()
        radio_box.setLayout(radio_box_layout)
        autonomous = QRadioButton("Autonomous")
        autonomous.setChecked(True)
        autonomous.toggled.connect(lambda checked: set_robot_state(State.AUTO))
        teleop = QRadioButton("Teleop")
        teleop.toggled.connect(lambda checked: set_robot_state(State.TELEOP))
        test = QRadioButton("Test")
        test.toggled.connect(lambda checked: set_robot_state(State.TEST))
        radio_box_layout.addWidget(autonomous, 0, 0)
        radio_box_layout.addWidget(teleop, 1, 0)
        radio_box_layout.addWidget(test, 2, 0)

        left_buttons = QWidget()
        right_buttons = QWidget()
        left_buttons_layout = QGridLayout()
        right_buttons_layout = QGridLayout()
        left_buttons.setLayout(left_buttons_layout)
        right_buttons.setLayout(right_buttons_layout)

        left_button_1 = QPushButton("0")
        left_button_1.pressed.connect(lambda i=0: press_driver(i))
        left_button_1.released.connect(lambda i=0: release_driver(i))
        left_button_2 = QPushButton("1")
        left_button_2.pressed.connect(lambda i=1: press_driver(i))
        left_button_2.released.connect(lambda i=1: release_driver(i))
        right_button_1 = QPushButton("2")
        right_button_1.pressed.connect(lambda i=2: press_driver(i))
        right_button_1.released.connect(lambda i=2: release_driver(i))
        right_button_2 = QPushButton("3")
        right_button_2.pressed.connect(lambda i=3: press_driver(i))
        right_button_2.released.connect(lambda i=3: release_driver(i))

        left_buttons_layout.addWidget(left_button_1, 0, 0)
        left_buttons_layout.addWidget(left_button_2, 0, 1)
        right_buttons_layout.addWidget(right_button_1, 0, 0)
        right_buttons_layout.addWidget(right_button_2, 0, 1)

        left_slider = QWidget()
        left_slider_layout = QGridLayout()
        left_slider.setLayout(left_slider_layout)
        left_slider_bar = QSlider(Qt.Horizontal)
        left_slider_bar.setFixedSize(150, 20)
        left_slider_bar.setMinimum(-20)
        left_slider_bar.setMaximum(20)
        left_slider_bar.valueChanged.connect(lambda value: set_left_twist(value))
        left_slider_bar.sliderReleased.connect(lambda button=left_slider_bar: release_slider(button))
        left_slider_layout.addWidget(left_slider_bar)

        right_slider = QWidget()
        right_slider_layout = QGridLayout()
        right_slider.setLayout(right_slider_layout)
        right_slider_bar = QSlider(Qt.Horizontal)
        right_slider_bar.setFixedSize(150, 20)
        right_slider_bar.setMinimum(-20)
        right_slider_bar.setMaximum(20)
        right_slider_bar.valueChanged.connect(lambda value: set_right_twist(value))
        right_slider_bar.sliderReleased.connect(lambda button=right_slider_bar: release_slider(button))
        right_slider_layout.addWidget(right_slider_bar)

        xbox_sticks = QWidget()
        xbox_sticks_layout = QGridLayout()
        xbox_sticks.setLayout(xbox_sticks_layout)

        left_stick = UIJoystick()
        right_stick = UIJoystick()

        xbox_left_stick = UIJoystick()
        xbox_right_stick = UIJoystick()

        xbox_left_trigger = QSlider(Qt.Horizontal)
        xbox_left_trigger.setFixedSize(150, 20)
        xbox_left_trigger.setMinimum(0)
        xbox_left_trigger.setMaximum(20)
        xbox_left_trigger.valueChanged.connect(lambda value: set_xbox_left_trigger(value))
        xbox_left_trigger.sliderReleased.connect(lambda button=xbox_left_trigger: release_slider(button))

        xbox_right_trigger = QSlider(Qt.Horizontal)
        xbox_right_trigger.setFixedSize(150, 20)
        xbox_right_trigger.setMinimum(0)
        xbox_right_trigger.setMaximum(20)
        xbox_right_trigger.valueChanged.connect(lambda value: set_xbox_right_trigger(value))
        xbox_right_trigger.sliderReleased.connect(lambda button=xbox_right_trigger: release_slider(button))

        xbox_sticks_layout.addWidget(xbox_left_stick, 0, 0)
        xbox_sticks_layout.addWidget(xbox_right_stick, 0, 1)
        xbox_sticks_layout.addWidget(xbox_left_trigger, 1, 0)
        xbox_sticks_layout.addWidget(xbox_right_trigger, 1, 1)

        xbox_button_box = QWidget()
        xbox_button_box_layout = QGridLayout()
        xbox_button_box.setLayout(xbox_button_box_layout)

        xbox_button_map = {
            0 : "A (0)",
            1 : "B (1)",
            2 : "X (2)",
            3 : "Y (3)",
            4 : "LB (4)",
            5 : "RB (5)",
            6 : "Select (6)",
            7 : "Start (7)",
            8 : "LS (8)",
            9 : "RS (9)",
        }

        for i in range(0, 10):
            button = QPushButton(xbox_button_map[i])
            button.pressed.connect(lambda index = i: press_xbox_button_box(index))
            button.released.connect(lambda index = i: release_xbox_button_box(index))
            xbox_button_box_layout.addWidget(button, i / 3, i % 3)

        button_box = QWidget()
        button_box_layout = QGridLayout()
        button_box.setLayout(button_box_layout)

        index = 0
        for j in range(0,4):
            for i in range(0,3):
                button = QPushButton(str(index))
                button.pressed.connect(lambda i=index: press_button_box(i))
                button.released.connect(lambda i=index: release_button_box(i))
                button_box_layout.addWidget(button, j, i)
                index += 1

        ml.addWidget(enable_disable_button, 0, 0)
        ml.addWidget(radio_box, 0, 1)
        if show_drive:
            ml.addWidget(left_stick, 1, 0)
            ml.addWidget(right_stick, 1, 1)
            ml.addWidget(left_slider, 2,0)
            ml.addWidget(right_slider, 2,1)
            ml.addWidget(left_buttons, 3, 0)
            ml.addWidget(right_buttons, 3, 1)
        if show_xbox:
            ml.addWidget(xbox_sticks, 1, 2)
            ml.addWidget(xbox_button_box, 2, 2)
        if show_button_box:
            ml.addWidget(button_box, 1, 3)

def set_xbox_left_trigger(value):
    global xbox_left_trigger
    xbox_left_trigger = value / 20.0

def set_xbox_right_trigger(value):
    global xbox_right_trigger
    xbox_right_trigger = value / 20.0

def release_slider(button):
    button.setValue(0)

def set_left_twist(value):
    global left_twist
    left_twist = value / 20.0

def set_right_twist(value):
    global right_twist
    right_twist = value / 20.0

def toggle_enable_disable(button):
    global robot_enabled
    robot_enabled = not robot_enabled
    if robot_enabled:
        button.setText("Disable")
        return
    button.setText("Enable")

def set_robot_state(state):
    global robot_state
    robot_state = state

def press_button_box(i):
    global button_box_buttons
    button_box_buttons[i] = 1

def release_button_box(i):
    global button_box_buttons
    button_box_buttons[i] = 0

def press_xbox_button_box(i):
    global xbox_button_box_buttons
    xbox_button_box_buttons[i] = 1

def release_xbox_button_box(i):
    global xbox_button_box_buttons
    xbox_button_box_buttons[i] = 0

def press_driver(i):
    global driver_buttons
    driver_buttons[i] = 1

def release_driver(i):
    global driver_buttons
    driver_buttons[i] = 0

def window():
    global app
    global widget

    app = QApplication(sys.argv)
    widget = MainWindow()

    widget.show()
    sys.exit(app.exec())

def ui_thread():
    window()

def ros_func():
    global hmi_updates
    global robot_status
    global left_stick
    global right_stick
    global xbox_left_stick
    global xbox_right_stick
    global xbox_left_trigger
    global xbox_right_trigger
    global button_box_buttons
    global driver_buttons
    global left_twist
    global right_twist

    rate = rospy.Rate(100)
    joystickStatusPublisher = rospy.Publisher(name='/JoystickSimulation', data_class=ck_ros_base_msgs_node.msg.Joystick_Status, queue_size=50, tcp_nodelay=True)
    robotStatusPublisher = rospy.Publisher(name='/RobotSimulation', data_class=ck_ros_base_msgs_node.msg.Robot_Status, queue_size=50, tcp_nodelay=True)

    # Put your code in the appropriate sections in this if statement/while loop
    while not rospy.is_shutdown():
        if robot_status.get_mode() == RobotMode.AUTONOMOUS:
            pass
        elif robot_status.get_mode() == RobotMode.TELEOP:
            pass
        elif robot_status.get_mode() == RobotMode.DISABLED:
            pass
        elif robot_status.get_mode() == RobotMode.TEST:
            pass

        if left_stick is not None and right_stick is not None:
            joystick_status = ck_ros_base_msgs_node.msg._Joystick_Status.Joystick_Status()
            driver_joystick = ck_ros_base_msgs_node.msg._Joystick.Joystick()
            driver_joystick.index = 0
            left_joystick_data = left_stick.joystickDirection()
            right_joystick_data = right_stick.joystickDirection()

            driver_joystick.axes.append(left_joystick_data[0])
            driver_joystick.axes.append(left_joystick_data[1])
            driver_joystick.axes.append(left_twist)
            driver_joystick.axes.append(right_joystick_data[0])
            driver_joystick.axes.append(right_joystick_data[1])
            driver_joystick.axes.append(right_twist)

            driver_joystick.buttons = driver_buttons

            joystick_status.joysticks.append(driver_joystick)

            xbox_joystick = ck_ros_base_msgs_node.msg._Joystick.Joystick()
            xbox_joystick.index = 1
            xbox_joystick.buttons = xbox_button_box_buttons
            xbox_joystick.axes.append(xbox_left_stick.joystickDirection()[0])
            xbox_joystick.axes.append(xbox_left_stick.joystickDirection()[1])
            xbox_joystick.axes.append(xbox_left_trigger)
            xbox_joystick.axes.append(xbox_right_trigger)
            xbox_joystick.axes.append(xbox_right_stick.joystickDirection()[0])
            xbox_joystick.axes.append(xbox_right_stick.joystickDirection()[1])

            joystick_status.joysticks.append(xbox_joystick)

            operator_joystick = ck_ros_base_msgs_node.msg._Joystick.Joystick()
            operator_joystick.index = 2
            operator_joystick.buttons = button_box_buttons
            joystick_status.joysticks.append(operator_joystick)

            joystickStatusPublisher.publish(joystick_status)

        override_robot_status = ck_ros_base_msgs_node.msg._Robot_Status.Robot_Status()
        if robot_enabled:
            if robot_state == State.AUTO:
                override_robot_status.robot_state = override_robot_status.AUTONOMOUS
            elif robot_state == State.TELEOP:
                override_robot_status.robot_state = override_robot_status.TELEOP
            else:
                override_robot_status.robot_state = override_robot_status.TEST
        else:
            override_robot_status.robot_state = override_robot_status.DISABLED

        override_robot_status.alliance = override_robot_status.RED
        override_robot_status.match_time = -1
        override_robot_status.game_data = ''
        override_robot_status.selected_auto = 0
        override_robot_status.is_connected = True

        robotStatusPublisher.publish(override_robot_status)

        rate.sleep()

def ros_spin():
    rospy.spin()

def ros_main(node_name):
    global button_box_buttons
    global driver_buttons

    rospy.init_node(node_name)
    register_for_robot_updates()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    for i in range(0,12):
        button_box_buttons.append(0)

    for i in range(0,10):
        xbox_button_box_buttons.append(0)

    for i in range(0,4):
        driver_buttons.append(0)

    t1 = Thread(target=ros_func)
    t1.start()

    t2 = Thread(target=ros_spin)
    t2.start()

    ui_thread()
    t1.join(5)
    t2.join(5)