#!/usr/bin/env python

''' Links used:
    https://stackoverflow.com/questions/8959815/restricting-the-value-in-tkinter-entry-widget
'''

# Import libraries
from __future__ import print_function, division
import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool
from Tkinter import *
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from PIL import ImageTk, Image

class GUI():

    def __init__(self, tk):

        # Constants
        self.sprite_location = "../images/"

        # continuous-update storage attributes
        self.estop_state = False
        self.activated_state = False
        self.debug_msg = None
        self.info_msg = None
        self.warn_msg = None
        self.error_msg = None

        # ROS Constructs
        rospy.init_node("gui")
        rospy.Subscriber("/rosout", Log, self.log_callback)
        rospy.Subscriber("/softestop", Bool, self.estop_callback)
        rospy.Subscriber("/state_controller/cmd_activate", Bool, self.activate_callback)
        self.estop_publisher = rospy.Publisher("/softestop", Bool, queue_size=1)
        self.activate_publisher = rospy.Publisher("/state_controller/cmd_activate", Bool, queue_size=1)

        # Create main container
        self.tk = tk
        self.tk.title("Main Tractor GUI")
        self.tk.geometry('{}x{}'.format(1250, 500))

        # layout main container
        self.tk.grid_rowconfigure(0, weight=1)
        self.tk.grid_columnconfigure(1, weight=1)

        # Create main sub-containers
        self.input_frame = Frame(self.tk, width=250, height=500, pady=3, padx=5)
        self.graph_frame = Frame(self.tk, width=1000, height=500, pady=3)

        # layout main sub-containers
        self.input_frame.grid(row=0, column=0, sticky="e")
        self.graph_frame.grid(row=0, column=1, sticky="w")

        # Create widgets for input frame
        self.estop_img = ImageTk.PhotoImage(Image.open(self.sprite_location + "estop.png"))
        self.estop_pressed_img = ImageTk.PhotoImage(Image.open(self.sprite_location + "estop_pressed.png"))
        self.button_estop = Button(self.input_frame, command=self.estop_command, image=self.estop_img)
        self.redled_img = ImageTk.PhotoImage(Image.open(self.sprite_location + "redled.png"))
        self.greenled_img = ImageTk.PhotoImage(Image.open(self.sprite_location + "greenled.png"))
        self.led_estop = Label(self.input_frame, image=self.greenled_img)
        self.activate_img = ImageTk.PhotoImage(Image.open(self.sprite_location + "estop.png"))
        self.disactivate_img = ImageTk.PhotoImage(Image.open(self.sprite_location + "estop_pressed.png"))
        self.button_activate = Button(self.input_frame, command=self.activate_command, image=self.activate_img)
        self.led_activate = Label(self.input_frame, image=self.redled_img)
        self.debug_img = ImageTk.PhotoImage(Image.open(self.sprite_location + "debug_window.png"))
        self.debug_display = Label(self.input_frame, text="DEBUG_DISPLAY", fg='black')
        self.info_display = Label(self.input_frame, text="INFO_DISPLAY", fg='black')
        self.warn_display = Label(self.input_frame, text="WARN_DISPLAY", fg='black')
        self.error_display = Label(self.input_frame, text="ERROR_DISPLAY")
        self.fatal_display = Label(self.input_frame, text="FATAL_DISPLAY")

        # layout widgets for input frame
        self.button_estop.grid(row=0, column=1, pady=5)
        self.led_estop.grid(row=1, column=1, pady=5)
        self.button_activate.grid(row=0, column=2, pady=5)
        self.led_activate.grid(row=1, column=2, pady=5)
        self.debug_display.grid(row=2, column=1, pady=5)
        self.info_display.grid(row=3, column=1, pady=5)
        self.warn_display.grid(row=4, column=1, pady=5)
        self.error_display.grid(row=5, column=1, pady=5)
        self.fatal_display.grid(row=6, column=1, pady=5)

    def log_callback(self, msg):
        '''
            @brief Update relevant message attribute with msg info
        '''
        try:
            if msg.level == 1:
                self.debug_display.configure(text=msg.msg)
            elif msg.level == 2:
                self.info_display.configure(text=msg.msg)
            elif msg.level == 4:
                self.warn_display.configure(text=msg.msg)
            elif msg.level == 8:
                self.error_display.configure(text=msg.msg)
            elif msg.level == 16:
                self.fatal_display.configure(text=msg.msg)
            else:
                print("Unknown ROS message id!")
        except AttributeError:
            print("screen not initialized yet")
        return

    def estop_callback(self, msg):
        # Updates interface with new estop info
        if msg.data != self.estop_state:
            self.update_estop_sprite(msg.data)
            self.estop_state = msg.data

    def activate_callback(self, msg):
        # Updates interface with new activation info
        if msg.data != self.activated_state:
            self.update_activate_sprite(msg.data)
            self.activated_state = msg.data

    def estop_command(self):
        # Toggles soft estopped state

        if not self.estop_state:
            self.estop_publisher.publish(Bool(True))
            self.estop_state = True
            self.activate_publisher.publish(Bool(False))
            self.activated_state = False
        else:
            self.estop_publisher.publish(Bool(False))
            self.estop_state = False

        self.update_estop_sprite(self.estop_state)
        self.update_activate_sprite(self.activated_state)

    def activate_command(self):
        # Toggles activated state

        if not self.activated_state and not self.estop_state:
            self.activate_publisher.publish(Bool(True))
            self.activated_state = True
        else:
            self.activate_publisher.publish(Bool(False))
            self.activated_state = False

        self.update_activate_sprite(self.activated_state)

    def update_estop_sprite(self, state):
        # Updates estop and led sprites

        if state:
            self.button_estop.configure(image=self.estop_pressed_img)
            self.led_estop.configure(image=self.redled_img)
        else:
            self.button_estop.configure(image=self.estop_img)
            self.led_estop.configure(image=self.greenled_img)

    def update_activate_sprite(self, state):
        # Updates activate and led sprites

        if state:
            self.button_activate.configure(image=self.disactivate_img)
            self.led_activate.configure(image=self.greenled_img)
        else:
            self.button_activate.configure(image=self.activate_img)
            self.led_activate.configure(image=self.redled_img)

    def validate(self, action, index, value_if_allowed, prior_value, text, validation_type, trigger_type, widget_name):
        if text in '0123456789.-+':
            try:
                float(value_if_allowed)
                return True
            except ValueError:
                return False
        else:
            return False

    def run(self):

        mainloop()

if __name__ == "__main__":

    tk = Tk()
    g = GUI(tk)
    g.run()
