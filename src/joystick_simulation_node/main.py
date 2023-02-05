#!/usr/bin/env python3

import tf2_ros
import rospy
from threading import Thread

from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode

import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot
from joystick_simulation_node.joystick import Joystick


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("My App")
        button = QPushButton("Press Me!")

        cw = QWidget()
        ml = QGridLayout()
        # ml.setSpacing(25)
        # ml.setContentsMargins(25, 25, 25, 25)
        cw.setLayout(ml)
        self.setCentralWidget(cw)

        self.left_stick = Joystick()
        self.right_stick = Joystick()

        ml.addWidget(self.left_stick, 0, 0)
        ml.addWidget(self.right_stick, 0, 1)

        # Set the central widget of the Window.

def window():
    app = QApplication(sys.argv)
    widget = MainWindow()

#    button1 = QPushButton()
#    button1.setText("Button1")
# #    button1.move(64,32)
#    button1.clicked.connect(button1_clicked)

#    button2 = QPushButton()
#    button2.setText("Button2")
# #    button2.move(64,64)
#    button2.clicked.connect(button2_clicked)

#    layout = QVBoxLayout()
#    layout.addWidget(button1)
#    layout.addWidget(button2)

#    widget.setGeometry(50,50,320,200)
#    widget.setWindowTitle("PyQt5 Button Click Example")

#    widget.setLayout(layout)

    widget.show()
    app.exec()

def button1_clicked():
   print("Button 1 clicked")

def button2_clicked():
   print("Button 2 clicked")

def ui_thread():
    window()


def ros_func():
    global hmi_updates
    global robot_status

    rate = rospy.Rate(20)
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

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    ui_thread()
    print("Got here at least")
    rospy.spin()

    t1.join(5)