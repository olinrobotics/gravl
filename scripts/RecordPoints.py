#!/usr/bin/env python

"""
Subscribes to `/tractor_odom` and starts a tkinter GUI with Record Point and
Save Data buttons. When Record Point is clicked, it saves the last 2D x-y
odometry point recieved. When Save Data is clicked, it writes the data to a
file that can be used in the cut_planner package.
"""

# import keyboard
import tkinter as tk
import rospy
import pandas as pd

from nav_msgs.msg import Odometry

class Recorder:
    def __init__(self):
        rospy.init_node("point_recorder")
        self.odom_sub = rospy.Subscriber("/tractor_odom", Odometry, self.odomCB)
        self.update_rate = rospy.Rate(10)

        self.root = tk.Tk()
        tk.Button(self.root, text="Record Point", command = self.ptCB).pack()
        tk.Button(self.root, text="Save Data", command = self.dataCB).pack()

        self.saved_x = []
        self.saved_y = []
        self.saved_z = []

        self.odom_msg = Odometry()

    def ptCB(self):
        print('Recording point')
        self.saved_x.append(self.odom_msg.pose.pose.position.x)
        self.saved_y.append(self.odom_msg.pose.pose.position.y)
        self.saved_z.append(self.odom_msg.pose.pose.position.z)

    def dataCB(self):
        """Write recorded points to an output text file

        TODO: make the output file a settable parameter
        """
        f = open("p2p_output.txt","w+")
        print("Saving data...")
        f.write("title: p2p_test\n")
        f.write("waypoints:\n")

        for i in range(len(self.saved_x)):
            f.write("   " + str(i) + ":\n")
            f.write("       index: " + str(i) + "\n")
            f.write("       point: {x: " + str(self.saved_x[i]) + \
                                 ", y: " + str(self.saved_y[i]) + \
                                 ", z: " + str(self.saved_z[i]) + \
                                 "}\n")
            f.write("       frame: odom\n")
            f.write("       behavior: p2p\n")
            f.write("       forward: true\n")
            f.write("       autocontinue: false\n")

        f.write("   " + str(len(self.saved_x)) + ":\n")
        f.write("       index: " + str(len(self.saved_x)) + "\n")
        f.write("       point: {x: 0, y: 0, z: 0}\n")
        f.write("       frame: odom\n")
        f.write("       behavior: safety\n")
        f.write("       forward: none\n")
        f.write("       autocontinue: none\n")

    def odomCB(self, msg):
        self.odom_msg = msg

    def run(self):
        while not rospy.is_shutdown():
            self.root.update()
            self.update_rate.sleep()

if __name__ == "__main__":
    rec = Recorder()
    rec.run()
