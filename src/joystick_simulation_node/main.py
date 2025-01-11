#!/usr/bin/env python3

import rospy
from threading import Thread
from collections import namedtuple
import signal
from sensor_msgs.msg import Joy
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.Qt import QTimer
from joystick_simulation_node.joystick import Joystick as UIJoystick
from frc_msgs.msg import JoystickState
from frc_msgs.msg import ButtonBoxState2024

BUTTON_COLORS = {
    "red": "#FF0000",
    "green": "#00FF00",
    "blue": "#0000FF",
    "yellow": "#FFFF00",
    "black": "#000000",
    "white": "#FFFFFF",
}
# This is starting to feel like a redundant wrapper around QPushButton
class Button():
    def __init__(self, name: str, color: str):
        self._q_push_button = QPushButton(name)
        if color is not None:
            self._q_push_button.setStyleSheet(f"background-color: {BUTTON_COLORS[color]}")

    def press(self, state: bool):
        self._q_push_button.setDown(state)

    def is_pressed(self):
        return self._q_push_button.isDown()

    def get_push_button(self):
        return self._q_push_button


# Create a slider control to simulate a joystick trigger
class Trigger():
    def __init__(self):
        self._q_slider = QSlider(Qt.Horizontal)
        self._q_slider.setFixedSize(150, 20)
        self._q_slider.setMinimum(0)
        self._q_slider.setMaximum(20)
        self._q_slider.valueChanged.connect(lambda value: self.set_trigger(value))
        self._q_slider.sliderReleased.connect(self.release_slider)
        self._value = 0.0

    def set_trigger(self, value):
        self._value = value / 20.0

    def release_slider(self):
        self._q_slider.setValue(0)

    def get_value(self):
        return self._value
    
    def set_value(self, value: float):
        self._value = value
        self._q_slider.setValue(value)


# Group 2 or more radio buttons together to act as a switch
class Switch():
    def __init__(self, names: list, initial_state: int = 0, momentary: bool = False):
        self._widget = QWidget()
        self._layout = QGridLayout()
        self._widget.setLayout(self._layout)
        self._q_radio_buttons = []
        for name in names:
            self._q_radio_buttons.append(QRadioButton(name))
            self._q_radio_buttons[-1].toggled.connect(lambda checked, state = len(self._q_radio_buttons) - 1: self._set_state_var(checked, state))
            self._layout.addWidget(self._q_radio_buttons[-1], len(self._q_radio_buttons) - 1, 0)   
        self._initial_state = initial_state
        self._momentary = momentary
        self._state = initial_state
        if momentary:
            self._timer = QTimer(self._widget)
            self._timer.timeout.connect(lambda: self.set_state(self._initial_state))
            self._timer.setSingleShot(True)
        if self._state < len(self._q_radio_buttons):
            self.set_state(initial_state)

    def _set_state_var(self, checked: bool, state: int):
        if checked:
            self._state = state
            if self._momentary and state != self._initial_state:
                self._timer.stop()
                self._timer.start(500)  

    def get_state(self):
        return self._state

    def set_state(self, state: int):
        self._q_radio_buttons[state].setChecked(True)

    def get_binary_state(self):
        return [1 if i == self._state else 0 for i in range(len(self._q_radio_buttons))]

    def get_widget(self):
        return self._widget

# Special case of a switch - 8-way POV switch
class POV(Switch):
    def __init__(self):
        super().__init__([""], initial_state=4, momentary=True) 
        names = ['U/L', 'U', 'U/R', 'L', 'C', 'R', 'D/L', 'D', 'D/R']
        self._q_radio_buttons.clear()
        for i in range(len(names)):
            self._q_radio_buttons.append(QRadioButton(names[i]))
            self._q_radio_buttons[-1].toggled.connect(lambda checked, state = len(self._q_radio_buttons) - 1: self._set_state_var(checked, state))
            self._layout.addWidget(self._q_radio_buttons[-1], int(i/3), i % 3)
        self.set_state(self._initial_state)

    def get_state(self):
        # First axis is left/right, with left being 1.0 and right being -1.0
        # Second axis is up/down, with up being 1.0 and down being -1.0
        state_to_axis = [
            [ 1.0,  1.0], # U/L
            [ 0.0,  1.0], # U
            [-1.0,  1.0], # U/R
            [ 1.0,  0.0], # L
            [ 0.0,  0.0], # C
            [-1.0,  0.0], # R
            [ 1.0, -1.0], # D/L
            [ 0.0, -1.0], # D
            [-1.0, -1.0], # D/R
        ]
        return state_to_axis[self._state]

# Input to select an integer auto mode
# This could be expanded to better match the actual robot's auto mode selection
# by having names modes and a dropdown? Or a text box which is updated based on the number?
# Or using the dial below plus feedback on the number and name?
# Lots of options
class AutoModeSpinner():
    def __init__(self, num_auto_buttons):
        self._q_spin_box = QSpinBox()
        self._q_spin_box.setRange(1, 20)
        self._q_spin_box.valueChanged.connect(self.set_value)
        self._value = 1
        self._num_auto_modes = num_auto_buttons

    def set_value(self, value: int):
        self._value = value

    def get_value(self):
        return self._value
    
    def get_widget(self):
        return self._q_spin_box

    def get_buttons(self):
        return [1 if i == self._value else 0 for i in range(self._num_auto_modes)]

''' Could potentially use a QDial for automode, previous code snippets :

        dial_box = QDial()
        dial_box_layout = QGridLayout()
        dial_box.setLayout(dial_box_layout)
        dial_box.setMinimum(0)
        dial_box.setMaximum(20)
        dial_box.setValue(0)
        dial_box.setRange(0,20)
        dial_box.setNotchesVisible(True)

        self.dial.valueChanged.connect(self.sliderMoved)
        layout.addWidget(self.dial)

    def sliderMoved(self):
        print("Dial value = %i" % (self.dial.value()))

'''

# Create a simulated controller
# This is a collection of various individual controls along with a layout for them
ControllerButton = namedtuple('ButtonBoxButton', ['name', 'color', 'joy_index', 'x_pos', 'y_pos'])
class Controller():
    def __init__(self):
        self._buttons = []
        self._button_joy_idx = [] # Which index in the joy message output does this button correspond to?
        self._sticks = []
        self._triggers = []
        self._switches = []
        self._switch_joy_idxs = []
        self._pov = None # Only 1 POV for now
        self._joystick_widget = QWidget()
        self._joysticks_layout = QGridLayout()
        self._joystick_widget.setLayout(self._joysticks_layout)   
        self._button_widget = QWidget()
        self._button_layout = QGridLayout()
        self._button_widget.setLayout(self._button_layout)
        self._auto_mode_spinner = None
        self._first_auto_button = None

    def add_button(self, name: str, color: str, joy_index: int, x_pos: int, y_pos: int):
        self._buttons.append(Button(name, color))
        self._button_layout.addWidget(self._buttons[-1].get_push_button(), x_pos, y_pos)
        self._button_joy_idx.append(joy_index)

    def add_stick(self):
        self._sticks.append(UIJoystick())
        i = len(self._sticks) - 1
        self._joysticks_layout.addWidget(self._sticks[i], int(i / 2), int (i % 2))

    def add_switch(self, names: list, joy_indexes: list, x_pos: int, y_pos: int, initial_state: int = 0, momentary: bool = False):
        self._switches.append(Switch(names, initial_state=initial_state, momentary=momentary))
        self._button_layout.addWidget(self._switches[-1].get_widget(), x_pos, y_pos)
        self._switch_joy_idxs.append(joy_indexes)

    def add_POV(self, x_pos: int, y_pos: int):
        self._pov = POV()
        self._button_layout.addWidget(self._pov.get_widget(), x_pos, y_pos)

    # TODO : Can this be done by passing in an object type to a combined add_stick/add_trigger method
    def add_trigger(self):
        self._triggers.append(Trigger())
        i = len(self._triggers) - 1
        self._joysticks_layout.addWidget(self._triggers[i]._q_slider, 1, i)

    def add_auto_mode_spinner(self, x_pos: int, y_pos: int, first_auto_button: int = 18, num_auto_buttons: int = 13):
        self._auto_mode_spinner = AutoModeSpinner(num_auto_buttons=num_auto_buttons)
        self._button_layout.addWidget(self._auto_mode_spinner.get_widget(), x_pos, y_pos)
        self._first_auto_button = first_auto_button

    # Build a Joy message button array for the buttons and switches in ths controller
    def get_buttons(self):
        if self._first_auto_button is None:
            max_idx = -1
            if len(self._button_joy_idx) > 0:
                max_idx = max(self._button_joy_idx)
            if len(self._switch_joy_idxs) > 0:
                # Lots of list comprehension to flatten out the list of lists of switch indexes
                max_idx = max(max_idx, max(i for s in self._switch_joy_idxs for i in s if i is not None))
            if max_idx == -1:
                return []
        else:
            max_idx = self._first_auto_button

        ret = [0 for _ in range(max_idx + 1)]
        for i in range(len(self._buttons)):
            ret[self._button_joy_idx[i]] = 1 if self._buttons[i].is_pressed() else 0

        for i in range(len(self._switches)):
            state = self._switches[i].get_binary_state()
            for j in range(len(state)):
                if self._switch_joy_idxs[i][j] is not None:
                    ret[self._switch_joy_idxs[i][j]] = state[j]

        if self._auto_mode_spinner is not None:
            ret += self._auto_mode_spinner.get_buttons()

        return ret

    def get_joystick_direction(self, index: int):
        return self._sticks[index].joystickDirection()

    def get_trigger_value(self, index: int):
        return self._triggers[index].get_value()

    def get_pov_state(self):
        if self._pov is None:
            return []
        return self._pov.get_state()

    def get_joysticks_widget(self):
        return self._joystick_widget
    
    def get_buttons_widget(self):
        return self._button_widget  

    def get_switches_widgets(self):
        return [s.get_widget() for s in self._switches]

    def set_joystick(self, index: int, x: float, y: float):
        if x == 0.0 and y == 0.0:
            self._sticks[index].set_zero()
        else:
            # Scale the -1.0->1.0 range to -100->100 to match the joystick widget's scaling
            self._sticks[index].move_joystick(x * 100, y * -100)

    def set_button(self, index: int, state: bool):
        self._buttons[index].press(state)

    def set_switch(self, index: int, state: int):
        self._switches[index].set_state(state)


xbox_controller = None
button_box_controller = None

class MainWindow(QMainWindow):
    def __init__(self, show_xbox, show_button_box):
        global xbox_controller
        global button_box_controller

        super().__init__()

        self.setWindowTitle('Joystick Simulator')
        icon = self.style().standardIcon(QStyle.SP_TitleBarMenuButton)
        self.setWindowIcon(icon)
        #Formatting

        cw = QWidget()
        ml = QGridLayout()
        cw.setLayout(ml)
        self.setCentralWidget(cw)
        
        if show_button_box:
            # Assume a 5 wide by 10 high grid. Many of the locations don't contain buttons,
            # but it should give space to get a mostly accurate layout
            # Tuple is name, joy_index, x_pos, y_pos
            button_box_buttons = [
                ControllerButton("Climb", "blue", 0, 5, 0),
                ControllerButton("Zero", "white", 1, 0, 0),
                ControllerButton("AmpAlign", "yellow", 2, 9, 0),
                ControllerButton("Outtake", "red", 10, 5, 4),
                ControllerButton("Preempt", "red", 11, 5, 2),
                ControllerButton("Fast", "red", 13, 6, 4),
                ControllerButton("Top", "green", 15, 2, 1),
                ControllerButton("Bottom", "green", 8, 2, 3),
                ControllerButton("Right", "green", 16, 1, 2),
                ControllerButton("Left", "green", 9, 3, 2),
                ControllerButton("Trap", "white", 17, 6, 0),
            ]

            button_box_controller = Controller()
            for b in button_box_buttons:
                button_box_controller.add_button(b.name, b.color, b.joy_index, b.x_pos, b.y_pos)

            # Switches are : [names], [joy_idxs], x_pos, y_pos, optional initial_state index
            button_box_controller.add_switch(["ShooterUp", "ShooterMid", "ShooterDown"], [3, None, 7], 9, 2, initial_state=1, momentary=True)
            button_box_controller.add_switch(["SpeedUp", "SpeedMid", "SpeedDown"], [5, None, 4], 9, 3, initial_state=1, momentary=True)
            button_box_controller.add_switch(["Not Safe", "Safe"], [None, 6], 9, 4, initial_state=1)

            # Auto mode buttons are 1-hot encoded starting from index 19
            button_box_controller.add_auto_mode_spinner(0, 4, first_auto_button=18, num_auto_buttons=13)

            # Add button box to the main window layout
            ml.addWidget(button_box_controller.get_buttons_widget(), 1, 1)

        if show_xbox:
            # Create a simulated xbox controller
            # Two joysticks, two triggers, plus buttons
            xbox_controller = Controller()
            # TODO : can these just be counts in the constructor?
            xbox_controller.add_stick() # Left stick
            xbox_controller.add_stick() # Right stick
            xbox_controller.add_trigger() # Left trigger
            xbox_controller.add_trigger() # Right trigger
            xbox_controller.add_POV(3, 2)

            #sets up buttons as well for the xbox button map
            # Tuple is name, joy_index, x_pos, y_pos
            xbox_buttons = [
                ControllerButton("A", "green", 1, 4, 6),
                ControllerButton("B", "red", 0, 3, 7),
                ControllerButton("X", "blue", 3, 3, 5),
                ControllerButton("Y", "yellow", 2, 2, 6),
                ControllerButton("LB", None, 4, 1, 2),
                ControllerButton("RB", None, 5, 1, 6),
                ControllerButton("Back", None, 6, 1, 3),
                ControllerButton("Start", None, 7, 1, 4),
                ControllerButton("Left Stick", None, 8, 0, 2),
                ControllerButton("Right Stick", None, 9, 0, 6)
            ]

            for b in xbox_buttons:
                xbox_controller.add_button(b.name, b.color, b.joy_index, b.x_pos, b.y_pos)

            ml.addWidget(xbox_controller.get_joysticks_widget(), 1, 3)
            ml.addWidget(xbox_controller.get_buttons_widget(), 2, 3)


def joystick_callback(msg: JoystickState):
    global xbox_controller

    xbox_controller.set_joystick(0, msg.leftStickX, msg.leftStickY)
    xbox_controller.set_joystick(1, msg.rightStickX, msg.rightStickY)

    xbox_controller.set_button(0, msg.buttonA)
    xbox_controller.set_button(1, msg.buttonB)
    xbox_controller.set_button(2, msg.buttonX)
    xbox_controller.set_button(3, msg.buttonY)
    xbox_controller.set_button(4, msg.bumperLeft)
    xbox_controller.set_button(5, msg.bumperRight)
    xbox_controller.set_button(6, msg.buttonBack)
    xbox_controller.set_button(7, msg.buttonStart)


def button_box_callback(msg: ButtonBoxState2024):
    #The callback for the button box
    #Subscribes to the button box topic and sets buttons on the GUI from the state in the msg
    global button_box_controller
    button_box_controller.set_button(0, msg.climbButton)
    button_box_controller.set_button(1, msg.zeroButton)
    button_box_controller.set_button(2, msg.subwooferShootButton)
    button_box_controller.set_button(8, msg.bottomGreenButton)
    button_box_controller.set_button(9, msg.leftGreenButton)
    button_box_controller.set_button(10, msg.backupButton1Button)
    button_box_controller.set_button(11, msg.redButton)
    button_box_controller.set_button(13, msg.backupButton2Button)
    button_box_controller.set_button(15, msg.topGreenButton)
    button_box_controller.set_button(16, msg.rightGreenButton)
    button_box_controller.set_button(17, msg.trapButton)

    # Shooter up / down
    if msg.shooterArmDownButton:
        button_box_controller.set_switch(0, 2)
    elif msg.shooterArmUpButton:
        button_box_controller.set_switch(0, 0)
    else:
        button_box_controller.set_switch(0, 1)
    
    # Speed up / down
    if msg.speedSwitchDownButton:
        button_box_controller.set_switch(1, 2)
    elif msg.speedSwitchUpButton:
        button_box_controller.set_switch(1, 0)
    else:
        button_box_controller.set_switch(1, 1)

    # Safe / not safe
    button_box_controller.set_switch(2, 1 if msg.safeButton else 0)


def ros_pub_function():
    global xbox_controller
    global button_box_controller

    rate = rospy.Rate(50)
    if xbox_controller is not None:
        xbox_controller_publisher = rospy.Publisher('/frcrobot_rio/js0_in', Joy, queue_size=1, tcp_nodelay=True)

    if button_box_controller is not None:
        button_box_publisher = rospy.Publisher('/frcrobot_rio/js1_in', Joy, queue_size=1, tcp_nodelay=True)

    xbox_joystick_msg = Joy()
    button_box_msg = Joy()
    
    while not rospy.is_shutdown():
        if xbox_controller is not None:
            xbox_left_stick_data = xbox_controller.get_joystick_direction(0)
            xbox_right_stick_data = xbox_controller.get_joystick_direction(1)
            xbox_left_trigger = xbox_controller.get_trigger_value(0)
            xbox_right_trigger = xbox_controller.get_trigger_value(1)

            # TODO - I'm really not sure what's causing the need for a - on the 2nd stick Y axis, but it seems
            # to be there for both sim and real joysticks. Figure out why this is true.
            xbox_joystick_msg.axes = [xbox_left_stick_data[0], xbox_left_stick_data[1], xbox_left_trigger, xbox_right_trigger, -xbox_right_stick_data[0], xbox_right_stick_data[1]] + xbox_controller.get_pov_state()
            xbox_joystick_msg.buttons = xbox_controller.get_buttons()

            xbox_controller_publisher.publish(xbox_joystick_msg)

        if button_box_controller is not None:
            button_box_msg.buttons = button_box_controller.get_buttons() #+ pov_buttons
            button_box_publisher.publish(button_box_msg)        
        rate.sleep()


def ros_spin():
    rospy.spin()


def ros_main(node_name):
    rospy.init_node(node_name)

    if rospy.has_param("~show_xbox"):
        show_xbox = rospy.get_param("~show_xbox")
    else:
        show_xbox = True

    if rospy.has_param("~show_button_box"):
        show_button_box = rospy.get_param("~show_button_box")
    else:
        show_button_box = True

    if rospy.has_param("~playback_mode"):
        playback_mode = rospy.get_param("~playback_mode")
    else:
        playback_mode = False

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Create the controller objects and the main window
    app = QApplication(sys.argv)
    widget = MainWindow(show_xbox, show_button_box)

    if playback_mode:
        if show_xbox:
            rospy.Subscriber("/frcrobot_rio/joystick_states1", JoystickState, joystick_callback)
        if show_button_box:
            rospy.Subscriber("/frcrobot_rio/button_box_states", ButtonBoxState2024, button_box_callback)
    else:
        t1 = Thread(target=ros_pub_function)
        t1.start()

    # Since both ros.spin() and app.exec() are blocking, we need to run them in separate threads
    t2 = Thread(target=ros_spin)
    t2.start()

    # QT magic. Show the app window and start the event loop
    widget.show()
    app.exec()

    if not playback_mode:
        t1.join(5)

    t2.join(5)
    