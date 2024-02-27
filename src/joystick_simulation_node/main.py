#!/usr/bin/env python3

import tf2_ros
import rospy
from threading import Thread
import os
import signal
from sensor_msgs.msg import Joy


import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from joystick_simulation_node.joystick import Joystick as UIJoystick
from frc_msgs.msg import JoystickState
from frc_msgs.msg import ButtonBoxState2023
from enum import Enum
joystick_state = JoystickState()
button_box_state = ButtonBoxState2023()






class Alliance(Enum):
    RED = 0
    BLUE = 1
class State(Enum):
    TOP = 1
    MID = 1
    BOT = 1

show_drive = True
show_xbox = True
show_button_box = True

left_stick = None
right_stick = None
xbox_left_stick = None
xbox_right_stick = None
xbox_joystick = None
button_box_buttons = []
xbox_button_box_buttons = []
driver_buttons = []
driver_stick_axes_xbox = []
app = None
widget = None
robot_state = State.TOP
robot_alliance = Alliance.RED
robot_enabled = False
left_twist = 0
right_twist = 0
xbox_left_trigger = 0
xbox_right_trigger = 0
pov_buttons = []

radio_buttons = []
radio_buttons_2 = []
radio_box_3_buttons = []

joystick_button_down = []
joystick_button_up = []

button_box_button_down = []
button_box_button_up = []

extra_pov_button_down = []
extra_pov_button_up = []

radio_test_button_check = []
radio_test_button_uncheck = []

xbox_left_joy = None
xbox_right_joy = None

xbox_l_l = None
xbox_l_r = None

xbox_r_l = None
xbox_r_r = None


class MainWindow(QMainWindow):
    def __init__(self):
        global show_drive
        global show_xbox
        global show_button_box
        global left_stick
        global right_stick
        global xbox_left_stick
        global xbox_right_stick

        global joystick_button_down
        global joystick_button_up

        global radio_test_button_check
        global radio_test_button_uncheck

        super().__init__()

        self.setWindowTitle('Joystick Simulator')
        icon = self.style().standardIcon(QStyle.SP_TitleBarMenuButton)
        self.setWindowIcon(icon)
        #Formatting

        cw = QWidget()
        ml = QGridLayout()
        cw.setLayout(ml)
        self.setCentralWidget(cw)

        enable_disable_button = QPushButton("Enable")
        enable_disable_button.clicked.connect(lambda checked, button=enable_disable_button: toggle_enable_disable(button))


        #Toggle Button definitions
        #These are the three switch states for the gui, there are three of them, the name can be changed to whatever you want by replacing the current text in the " " to whatever you desire.

        radio_box = QWidget()
        radio_box_layout = QGridLayout()
        radio_box.setLayout(radio_box_layout)
        autonomous = QRadioButton("Top")
        radio_test_button_check.append(autonomous)
        radio_test_button_uncheck.append(autonomous)
        
        autonomous.toggled.connect(lambda checked, state_index=0: set_robot_state(state_index))
        teleop = QRadioButton("Mid")
        radio_test_button_check.append(teleop)
        radio_test_button_uncheck.append(teleop)

        teleop.toggled.connect(lambda checked, state_index=1: set_robot_state(state_index))
        test = QRadioButton("Bot")
        radio_test_button_check.append(test)
        radio_test_button_uncheck.append(test)

        test.toggled.connect(lambda checked, state_index=2: set_robot_state(state_index))
        radio_box_layout.addWidget(autonomous, 0, 0)
        radio_box_layout.addWidget(teleop, 1, 0)
        radio_box_layout.addWidget(test, 2, 0)


        alliance_radio_box = QWidget()
        alliance_radio_box_layout = QGridLayout()
        alliance_radio_box.setLayout(alliance_radio_box_layout)
        top_1 = QRadioButton("t2")
        top_1.toggled.connect(lambda checked, state_index=0: set_robot_alliance(state_index))
        mid_1 = QRadioButton("m2")
        mid_1.toggled.connect(lambda checked, state_index=1: set_robot_alliance(state_index))
        bottom_1 = QRadioButton("b2")
        bottom_1.toggled.connect(lambda checked, state_index=2: set_robot_alliance(state_index))
        alliance_radio_box_layout.addWidget(top_1, 0, 0)
        alliance_radio_box_layout.addWidget(mid_1, 1, 0)
        alliance_radio_box_layout.addWidget(bottom_1, 2, 0)

        radio_box_3 = QWidget()
        radio_box_3_layout = QGridLayout()
        radio_box_3.setLayout(radio_box_3_layout)
        top_2 = QRadioButton("t3")
        top_2.toggled.connect(lambda checked, state_index=0: set_radio_3(state_index))
        mid_2 = QRadioButton("m3")
        mid_2.toggled.connect(lambda checked, state_index=1: set_radio_3(state_index))
        bot_2 = QRadioButton("b3")
        bot_2.toggled.connect(lambda checked, state_index=2: set_radio_3(state_index))
        radio_box_3_layout.addWidget(top_2, 0, 0)
        radio_box_3_layout.addWidget(mid_2, 1, 0)
        radio_box_3_layout.addWidget(bot_2, 2, 0)



        
        #Button Creation


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
        
        #These buttons are for the controllers, we have no use for them so they are unbound.
        









        #These are the repurposed pov buttons.
        #They have been repurposed for the use of the button box.
        #These buttons can be found on the lower half of the gui.
        #These are currently unbound, if you would like to use them, set the according state in the callback msg to toggle or press down on hte indice of the 
        #pov button list. 

        #pov_button_0 = QPushButton("0")
        #pov_button_0.pressed.connect(lambda i=0: press_pov(i))
        #extra_pov_button_down.append(pov_button_0)
        #pov_button_0.released.connect(lambda i=0: release_pov(i))
        #extra_pov_button_up.append(pov_button_0)
#
        #pov_button_1 = QPushButton("1")
        #pov_button_1.pressed.connect(lambda i=1: press_pov(i))
        #extra_pov_button_down.append(pov_button_1)
        #pov_button_1.released.connect(lambda i=1: release_pov(i))
        #extra_pov_button_up.append(pov_button_1)
#
        #pov_button_2 = QPushButton("2")
        #pov_button_2.pressed.connect(lambda i=2: press_pov(i))
        #extra_pov_button_down.append(pov_button_2)
        #pov_button_2.released.connect(lambda i=2: release_pov(i))
        #extra_pov_button_up.append(pov_button_2)

        pov_button_3 = QPushButton("white_button")
        pov_button_3.pressed.connect(lambda i=0: press_pov(i))
        extra_pov_button_down.append(pov_button_3)
        pov_button_3.released.connect(lambda i=0: release_pov(i))
        extra_pov_button_up.append(pov_button_3)

        pov_button_4 = QPushButton("W")
        pov_button_4.pressed.connect(lambda i=1: press_pov(i))
        extra_pov_button_down.append(pov_button_4)
        pov_button_4.released.connect(lambda i=1: release_pov(i))
        extra_pov_button_up.append(pov_button_4)

        #pov_button_5 = QPushButton("5")
        #pov_button_5.pressed.connect(lambda i=5: press_pov(i))
        #extra_pov_button_down.append(pov_button_5)
        #pov_button_5.released.connect(lambda i=5: release_pov(i))
        #extra_pov_button_up.append(pov_button_5)

        pov_button_6 = QPushButton("A")
        pov_button_6.pressed.connect(lambda i=2: press_pov(i))
        extra_pov_button_down.append(pov_button_6)
        pov_button_6.released.connect(lambda i=2: release_pov(i))
        extra_pov_button_up.append(pov_button_6)

        pov_button_7 = QPushButton("S")
        pov_button_7.pressed.connect(lambda i=3: press_pov(i))
        extra_pov_button_down.append(pov_button_7)
        pov_button_7.released.connect(lambda i=3: release_pov(i))
        extra_pov_button_up.append(pov_button_7)

        pov_button_8 = QPushButton("D")
        pov_button_8.pressed.connect(lambda i=4: press_pov(i))
        extra_pov_button_down.append(pov_button_8)
        pov_button_8.released.connect(lambda i=4: release_pov(i))
        extra_pov_button_up.append(pov_button_8)







        left_buttons_layout.addWidget(left_button_1, 0, 0)
        left_buttons_layout.addWidget(left_button_2, 0, 1)
       
        right_buttons_layout.addWidget(right_button_1, 0, 0)
        right_buttons_layout.addWidget(right_button_2, 0, 1)

        left_slider = QWidget()
        left_slider_layout = QGridLayout()
        left_slider.setLayout(left_slider_layout)
        left_slider_bar = QSlider(Qt.Horizontal)
        left_slider_bar.setFixedSize(150, 20)
        left_slider_bar.setMinimum(-1)
        left_slider_bar.setMaximum(1)
        left_slider_bar.valueChanged.connect(lambda value: set_left_twist(value))
        left_slider_layout.addWidget(left_slider_bar)

        right_slider = QWidget()
        right_slider_layout = QGridLayout()
        right_slider.setLayout(right_slider_layout)
        right_slider_bar = QSlider(Qt.Horizontal)
        right_slider_bar.setFixedSize(150, 20)
        right_slider_bar.setMinimum(int(-1))
        right_slider_bar.setMaximum(int(1))
        right_slider_bar.valueChanged.connect(lambda value: set_right_twist(value))
        right_slider_bar.sliderReleased.connect(lambda button=right_slider_bar: release_slider(button))
        right_slider_layout.addWidget(right_slider_bar)

        xbox_sticks = QWidget()
        xbox_sticks_layout = QGridLayout()
        xbox_sticks.setLayout(xbox_sticks_layout)

        left_stick = UIJoystick()
        right_stick = UIJoystick()
        #pov = QWidget()

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
            6 : "BB (6)",
            7 : "Start (7)",
        }
        #sets up buttons as well for the xbox button map


        #Sets up the buttons from the joystick inputs.
        for i in range(0, 8):
            button = QPushButton(xbox_button_map[i])
            button.pressed.connect(lambda index = i: press_xbox_button_box(index))
            joystick_button_down.append(button)
            button.released.connect(lambda index = i: release_xbox_button_box(index))
            joystick_button_up.append(button)
            xbox_button_box_layout.addWidget(button, int(i / 3), int(i % 3))






    

        button_box = QWidget()
        button_box_layout = QGridLayout()
        button_box.setLayout(button_box_layout)


        
        #sets up buttons from 0-11


        #Sets up individual buttons on the button box.

        #index = 0
        #for j in range(0,4):
        #    for i in range(0,3):
        #        button = QPushButton(str(index))
        #        button.pressed.connect(lambda i=index: press_button_box(i))
        #        button_box_button_down.append(button) #creates list for the button down, 12 items
        #        button.released.connect(lambda i=index: release_button_box(i))
        #        button_box_button_up.append(button) #creates list for button up, 12 items
        #        button_box_layout.addWidget(button, j, i)
        #        index += 1


        #middl pack of buttons that we are instantiating indivudally ^


        '''
        pov_button_3 = QPushButton("white_button")
        pov_button_3.pressed.connect(lambda i=3: press_pov(i))
        extra_pov_button_down.append(pov_button_3)
        pov_button_3.released.connect(lambda i=3: release_pov(i))
        extra_pov_button_up.append(pov_button_3)
        '''




        bottom_button_1 = QPushButton("circle_blue_button")
        bottom_button_1.pressed.connect(lambda i=5: press_pov(i))
        extra_pov_button_down.append(bottom_button_1)
        bottom_button_1.released.connect(lambda i=5: release_pov(i))
        extra_pov_button_up.append(bottom_button_1)

        bottom_button_2 = QPushButton("square_red_button")
        bottom_button_2.pressed.connect(lambda i=6: press_pov(i))
        extra_pov_button_down.append(bottom_button_2)
        bottom_button_2.released.connect(lambda i=6: release_pov(i))
        extra_pov_button_up.append(bottom_button_2)


        bottom_button_3 = QPushButton("circle_red_button_top")
        bottom_button_3.pressed.connect(lambda i=7: press_pov(i))
        extra_pov_button_down.append(bottom_button_3)
        bottom_button_3.released.connect(lambda i=7: release_pov(i))
        extra_pov_button_up.append(bottom_button_3)


        bottom_button_4 = QPushButton("circle_white_button")
        bottom_button_4.pressed.connect(lambda i=8: press_pov(i))
        extra_pov_button_down.append(bottom_button_4)
        bottom_button_4.released.connect(lambda i=8: release_pov(i))
        extra_pov_button_up.append(bottom_button_4)


        bottom_button_5 = QPushButton("circle_red_button_bottom")
        bottom_button_5.pressed.connect(lambda i=9: press_pov(i))
        extra_pov_button_down.append(bottom_button_5)
        bottom_button_5.released.connect(lambda i=9: release_pov(i))
        extra_pov_button_up.append(bottom_button_5)


        bottom_button_6 = QPushButton("circle_yellow_button")
        bottom_button_6.pressed.connect(lambda i=10: press_pov(i))
        extra_pov_button_down.append(bottom_button_6)
        bottom_button_6.released.connect(lambda i=10: release_pov(i))
        extra_pov_button_up.append(bottom_button_6)

        #button_box_layout.addWidget(pov_button_0, 5, 0)
        #button_box_layout.addWidget(pov_button_1, 5, 1)
        #button_box_layout.addWidget(pov_button_2, 5, 2)
        button_box_layout.addWidget(pov_button_3, 1, 0)
        button_box_layout.addWidget(pov_button_4, 1, 1)
        button_box_layout.addWidget(pov_button_6, 2, 0)
        button_box_layout.addWidget(pov_button_7, 2, 1)
        button_box_layout.addWidget(pov_button_8, 2, 2)

        button_box_layout.addWidget(bottom_button_1, 3, 0)
        button_box_layout.addWidget(bottom_button_2, 3, 1)
        button_box_layout.addWidget(bottom_button_3, 3, 2)
        button_box_layout.addWidget(bottom_button_4, 4, 0)
        button_box_layout.addWidget(bottom_button_5, 4, 2)
        button_box_layout.addWidget(bottom_button_6, 5, 0)

        #Sets up locations for the button box.

        ml.addWidget(radio_box, 0, 1)
        ml.addWidget(alliance_radio_box, 0, 2)
        ml.addWidget(radio_box_3, 0, 3)
        if show_drive:
            ml.addWidget(left_stick, 1, 0)
            ml.addWidget(right_stick, 1, 1)
            ml.addWidget(left_slider, 2,0)
            ml.addWidget(right_slider, 2,1)
            ml.addWidget(left_buttons, 3, 0)
            ml.addWidget(right_buttons, 3, 1)
        if show_button_box:
            ml.addWidget(button_box, 1, 2)
        if show_xbox:
            ml.addWidget(xbox_sticks, 1, 3)
            ml.addWidget(xbox_button_box, 2, 3)






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

def set_robot_state(i):
    global radio_buttons
    radio_buttons = [0, 0, 0]
    radio_buttons[i] = 1

def set_robot_alliance(i):
    global radio_buttons_2
    radio_buttons_2 = [0,0,0]
    radio_buttons_2[i] = 1

def set_radio_3(i):
    global radio_box_3_buttons
    radio_box_3_buttons = [0,0,0]
    radio_box_3_buttons[i] = 1


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

def press_pov(i):
    global pov_buttons
    pov_buttons[i] = 1

def release_pov(i):
    global pov_buttons
    pov_buttons[i] = 0



#Callbacks below
#Whenever the subscriber picks up something it calls these callbacks below.

def joystick_controller_callback(msg):
    global joystick_button_down
    global joystick_button_up
    global xbox_left_joy
    global xbox_left_stick
    global xbox_right_stick


    #xbox_left_joy.move_joystick(msg.leftStickX, msg.leftStickY)
    #stimulates joystick for a preset condition
    #make sure that passed inputs to said function move_joystick, have a inverted y value, that is, the y value being passed into the functino is negative.
    #This is due to the way that the gui is setup.



    #The lines below with relation to the left and right stick allow for the gui to display the rosbag input from the joysticks itself.
    #magnification here exists due to the way that the gui is setup in the joystick.py file, it allows us to exaggerate the joystick movement graphically so that 
    #changes in movement are visible.
    if (msg.leftStickX != (0.0) and msg.leftStickY != (0.0)):
       
        magnify_lx = msg.leftStickX * 100
        magnify_ly = msg.leftStickY * -100
        xbox_left_stick.move_joystick(magnify_lx, magnify_ly)
        #print(magnify_x, magnify_y)
    else:
        xbox_left_stick.set_zero()

    if (msg.rightStickX != (0.0) and msg.rightStickY != (0.0)):
        magnify_rx = msg.rightStickX * 100
        magnify_ry = msg.rightStickX * -100
        xbox_right_stick.move_joystick(magnify_rx, magnify_ry)
    else:
        xbox_right_stick.set_zero()


    


    #For each button with relation to the controllers, these if statements make it so that buttons pressed in the rosbag, are displayed visually in the gui as well.
    #The purppose of the press and release functions is for the publishers which publish the input that you put into the joystick sim gui.
    #While the setdown methods allow for the gui to change accordingly to the rosbag, ex, if a button is pressed down, the gui indicates the button as being pressed down.
    #All buttons have a index, refer to the index mapping for the controller.
    #
    #Also make sure to read the msg file that is in use for the appropriate rosbag. The buttons should be set up accordingly.

    '''
      0 : "A (0)",
            1 : "B (1)",
            2 : "X (2)",
            3 : "Y (3)",
            4 : "LB (4)",
            5 : "RB (5)",
            6 : "BB (6)",
            7 : "Start (7)",
    '''

    if msg.buttonAPress == True:
        press_xbox_button_box(0)
        joystick_button_down[0].setDown(True)
    if msg.buttonARelease == True:
        release_xbox_button_box(0)
        joystick_button_down[0].setDown(False)


    if msg.buttonBPress == True:
        press_xbox_button_box(1)
        joystick_button_down[1].setDown(True)
    if msg.buttonBRelease == True:
        release_xbox_button_box(1)
        joystick_button_down[1].setDown(False)


    if msg.buttonXPress == True:
        press_xbox_button_box(2)
        joystick_button_down[2].setDown(True)
    if msg.buttonXRelease == True:
        release_xbox_button_box(2)
        joystick_button_down[2].setDown(False)

    if msg.buttonYPress == True:
        press_xbox_button_box(3)
        joystick_button_down[3].setDown(True)
    if msg.buttonYRelease == True:
        release_xbox_button_box(3)
        joystick_button_down[3].setDown(False)


    if msg.bumperLeftPress == True:
        press_xbox_button_box(4)
        joystick_button_down[4].setDown(True)
    if msg.bumperLeftRelease == True:
        release_xbox_button_box(4)
        joystick_button_down[4].setDown(False)


    if msg.bumperRightPress == True:
        press_xbox_button_box(5)
        joystick_button_down[5].setDown(True)
    if msg.bumperRightRelease == True:
        release_xbox_button_box(5)
        joystick_button_down[5].setDown(False)


    if msg.buttonBackPress == True:
        press_xbox_button_box(6)
        joystick_button_down[6].setDown(True)
    if msg.buttonBackRelease == True:
        release_xbox_button_box(6)
        joystick_button_down[6].setDown(False)


    if msg.buttonStartPress == True:
        press_xbox_button_box(7)
        joystick_button_down[7].setDown(True)
    if msg.buttonStartRelease == True:
        release_xbox_button_box(7)
        joystick_button_down[7].setDown(False)

        '''
        The list of if statements above exist so as to ensure that the data being subscribed to is synced with the gui itself.
        That is, if the button is set down in the rosbag, the gui shows this accordingly and vice versa.

        Note that the press and release functions exist so as to publish the output that is put onto the gui itself. 

        '''




def joystick_callback(data):
        global joystick_state
        joystick_state = data
        global xbox_left_joy

        joystick_controller_callback(joystick_state) 

def button_box_controller_callback(msg):
    #The callback for the button box
    #Subscribes to the button box topic and acts accordingly given the rosbag input.
    #Note that here we are use the .setChecked method, this allows for the graphical maniuplation of the three switch states.
    #In which, we have set up radio buttons to show the three switch state graphically.
    global radio_test_button_check
    global radio_test_button_uncheck
    if msg.heightSelectSwitchLeftButton == True:
        radio_test_button_check[0].setChecked(True)
    
    if msg.heightSelectSwitchRightButton == True:
        radio_test_button_check[2].setChecked(True)

    if msg.heightSelectSwitchLeftButton == False and msg.heightSelectSwitchRightButton == False:
        radio_test_button_check[1].setChecked(True)

    


#add button box controller callback functions into here.
        
'''
Msg files has all of the defined msg types, all that is left is to set up th ebutton box indices saccoirndlgy. 

Note that there are functions for both pressing and releasing the individual indices on the buttonbox itself. 
This is for the topic that is publishing to take in and pass onto the topic. 
The method on the arrays however, known as "setDown" or "setChecked" configures the button accordingly on the gui to be indicated as checked or setdown.
These have two states as seen above, true and false.
'''



'''
Within the button_bo_controller_callback we need to add the rest of hte states taht are accounted for using hte buttonbox msg types.

'''
    
    

def button_box_callback(data):
    global button_box_state
    button_box_state = data
    button_box_controller_callback(button_box_state)
  

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
    global pov_buttons
    global xbox_joystick
    global driver_stick_axes_xbox

    global xbox_left_joy
    global xbox_right_joy
    rate = rospy.Rate(100)
    DriverStickPublisher = rospy.Publisher('/frcrobot_rio/js0_in', Joy, queue_size=50, tcp_nodelay=True)
    SecondaryPublisher = rospy.Publisher('/frcrobot_rio/js2_in', Joy, queue_size=50, tcp_nodelay=True)
    OperartorBoxPublisher = rospy.Publisher('/frcrobot_rio/js1_in', Joy, queue_size=50, tcp_nodelay=True)
    
    rospy.Subscriber("/frcrobot_rio/joystick_states1", JoystickState, joystick_callback)
    rospy.Subscriber("/frcrobot_rio/button_box_states", ButtonBoxState2023, button_box_callback)

    xbox_joystick = Joy()
    #driver_joystick = Joy()
    operator_buttons = Joy()
    #instead of appending values, we should just put values into the list indvidually.

    

    # Put your code in the appropriate sections in this if statement/while loop
    xbox_joystick.axes = driver_stick_axes_xbox

    
    while not rospy.is_shutdown():
        #larva = Joy()
        #rospy.Rate(100)
        if left_stick is not None and right_stick is not None:
        #    joystick_status = Joy()
    
            
    
            #left_joystick_data = left_stick.joystickDirection()
            #right_joystick_data = right_stick.joystickDirection()

            xbox_left_stick_data = xbox_left_stick.joystickDirection()
            xbox_right_stick_data = xbox_right_stick.joystickDirection()

            xbox_left_joy = xbox_left_stick
            xbox_right_joy = xbox_right_stick


            #the lines above repreateedly set the direction equal to the referenced object, this allows us to update the positions in real time via the loop
#
            #driver_joystick.axes.append(left_joystick_data[0])
            #driver_joystick.axes.append(left_joystick_data[1])
            #driver_joystick.axes.append(left_twist)
            #driver_joystick.axes.append(right_joystick_data[0])
            #driver_joystick.axes.append(right_joystick_data[1])
            #driver_joystick.axes.append(right_twist)
            #dont have a secondary joystick so the code above is not necessary.
            
            #driver_joystick.buttons = driver_buttons
            #also isn't necessary
        
            xbox_joystick.axes = [xbox_left_stick_data[0], xbox_left_stick_data[1], xbox_left_trigger, xbox_right_stick_data[0], xbox_right_stick_data[1], xbox_right_trigger]
            #the line above ensures that for each loop, we update topic js0 accordingly with the values of the joystick.

            xbox_joystick.buttons = xbox_button_box_buttons
            
            operator_buttons.buttons = button_box_buttons + pov_buttons + radio_buttons + radio_buttons_2 + radio_box_3_buttons

            OperartorBoxPublisher.publish(operator_buttons)        
            #SecondaryPublisher.publish(driver_joystick)
            DriverStickPublisher.publish(xbox_joystick)
        rate.sleep()


def ros_spin():
    rospy.spin()

def ros_main(node_name):
    global button_box_buttons
    global driver_buttons
    global xbox_button_box_buttons
    global pov_buttons
    global radio_buttons
    global radio_buttons_2
    global radio_box_3_buttons
    global driver_stick_axes_xbox
    
    rospy.init_node(node_name)


    signal.signal(signal.SIGINT, signal.SIG_DFL)

    for i in range(0,12):
        button_box_buttons.append(0)

    for i in range(0,10):
        xbox_button_box_buttons.append(0)

    for i in range(0,4):
        driver_buttons.append(0)
    2
    for i in range(0,14):
        pov_buttons.append(0)
    
    for i in range(0,3):
        radio_buttons.append(0)
    
    for i in range(0,3):
        radio_buttons_2.append(0)

    for i in range(0,3):
        radio_box_3_buttons.append(0)

    for i in range(0,6):
        driver_stick_axes_xbox.append(0)

    


    t1 = Thread(target=ros_func)
    t1.start()

    t2 = Thread(target=ros_spin)
    t2.start()

    ui_thread()
    t1.join(5)
    t2.join(5)
    