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
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot
from joystick_simulation_node.joystick import Joystick as UIJoystick
from ck_ros_base_msgs_node.msg import *

left_stick = None
right_stick = None
button_box_buttons = []
driver_buttons = []
app = None
widget = None

class MainWindow(QMainWindow):
    def __init__(self):
        global left_stick
        global right_stick
        super().__init__()

        self.setWindowTitle("My App")

        cw = QWidget()
        ml = QGridLayout()
        cw.setLayout(ml)
        self.setCentralWidget(cw)

        left_buttons = QWidget()
        right_buttons = QWidget()
        left_buttons_layout = QGridLayout()
        right_buttons_layout = QGridLayout()
        left_buttons.setLayout(left_buttons_layout)
        right_buttons.setLayout(right_buttons_layout)

        left_button_1 = QPushButton("0")
        left_button_1.mousePressEvent = lambda checked, i=0: press_driver(i)
        left_button_1.mouseReleaseEvent = lambda checked, i=0: release_driver(i)
        left_button_2 = QPushButton("1")
        left_button_2.mousePressEvent = lambda checked, i=1: press_driver(i)
        left_button_2.mouseReleaseEvent = lambda checked, i=1: release_driver(i)
        right_button_1 = QPushButton("2")
        right_button_1.mousePressEvent = lambda checked, i=2: press_driver(i)
        right_button_1.mouseReleaseEvent = lambda checked, i=2: release_driver(i)
        right_button_2 = QPushButton("3")
        right_button_2.mousePressEvent = lambda checked, i=3: press_driver(i)
        right_button_2.mouseReleaseEvent = lambda checked, i=3: release_driver(i)

        left_buttons_layout.addWidget(left_button_1, 0, 0)
        left_buttons_layout.addWidget(left_button_2, 0, 1)
        right_buttons_layout.addWidget(right_button_1, 0, 0)
        right_buttons_layout.addWidget(right_button_2, 0, 1)

        left_stick = UIJoystick()
        right_stick = UIJoystick()

        button_box = QWidget()
        button_box_layout = QGridLayout()
        button_box.setLayout(button_box_layout)

        index = 0
        for j in range(0,4):
            for i in range(0,3):
                button = QPushButton(str(index))
                button.mousePressEvent = lambda event, i=index: press_button_box(i)
                button.mouseReleaseEvent = lambda event, i=index: release_button_box(i)
                button_box_layout.addWidget(button, j, i)
                index += 1

        ml.addWidget(left_stick, 0, 0)
        ml.addWidget(right_stick, 0, 1)
        ml.addWidget(left_buttons, 1, 0)
        ml.addWidget(right_buttons, 1, 1)
        ml.addWidget(button_box, 0, 2)

def press_button_box(i):
    global button_box_buttons
    button_box_buttons[i] = 1

def release_button_box(i):
    global button_box_buttons
    button_box_buttons[i] = 0

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
    global button_box_buttons
    global driver_buttons

    rate = rospy.Rate(100)
    statusPublisher = rospy.Publisher(name='/JoystickSimulation', data_class=ck_ros_base_msgs_node.msg.Joystick_Status, queue_size=50, tcp_nodelay=True)

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
            driver_joystick.axes.append(right_joystick_data[0])
            driver_joystick.axes.append(right_joystick_data[1])

            driver_joystick.buttons = driver_buttons

            joystick_status.joysticks.append(driver_joystick)

            operator_joystick = ck_ros_base_msgs_node.msg._Joystick.Joystick()
            operator_joystick.index = 1
            operator_joystick.buttons = button_box_buttons
            joystick_status.joysticks.append(operator_joystick)

            statusPublisher.publish(joystick_status)

        rate.sleep()

def ros_spin():
    rospy.spin()

def term_handler(signal, frame):
    global app
    global widget

    if widget is not None:
        widget.close()
    if app is not None:
        app.quit()
        app.exit()
    os.kill(os.getpid(), 9)
    sys.exit()
    os._exit(0)

def ros_main(node_name):
    global button_box_buttons
    global driver_buttons

    rospy.init_node(node_name)
    register_for_robot_updates()

    signal.signal(signal.SIGINT, term_handler)

    for i in range(0,12):
        button_box_buttons.append(0)

    for i in range(0,4):
        driver_buttons.append(0)

    t1 = Thread(target=ros_func)
    t1.start()

    t2 = Thread(target=ros_spin)
    t2.start()

    ui_thread()
    t1.join(5)
    t2.join(5)