#!/usr/bin/python

import getopt, sys
import csv

import rospy
import numpy as np
import tkinter as tk
import message_filters
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import heron_gui.config as CFG

class Heron_GUI(tk.Tk):
    def __init__(self, mode):
        super().__init__()

        self.round_decimal_points = 7

        self.lla_data = [0.0, 0.0, 0.0]
        self.utm_data = [0.0, 0.0, 0.0]
        self.pos_data_prev = [0.0, 0.0, 0.0]
        self.pos_data = [0.0, 0.0, 0.0]

        # Origin (Left Bottom COrner of the image)
        self.pond_lla = CFG.MAP_ORIGIN_LLA
        self.pond_utm = CFG.MAP_ORIGIN_UTM # (m) of most left bottom corner

        self.local_x_arr = np.array([])
        self.local_y_arr = np.array([])
        self.waypoints = np.zeros((5,3))

        self.sub_lla = message_filters.Subscriber(CFG.GPS_SUBSCRIBER, NavSatFix)
        self.sub_utm = message_filters.Subscriber(CFG.UTM_SUBSCRIBER, PoseStamped)

        ts = message_filters.TimeSynchronizer([self.sub_lla, self.sub_utm], 10)
        ts.registerCallback(self.callback)
        
        topFrame = tk.Frame(self, width=1350, height=50)
        topFrame.pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # Window
        self.title = "Heron GPS Tracking & Waypoint Viewer GUI"
        self.label = ttk.Label(topFrame,
                            text="Latitude: {}\nLongitude: {}\nAltitude: {}\n\nUTM X: {}\nUTM Y: {}\nUTM Z: {}\n\nLocal X: {}\nLocal Y: {}\nLocal Z: {}" \
                                  .format(self.lla_data[0], self.lla_data[1], self.lla_data[2], \
                                          self.utm_data[0], self.utm_data[1], self.utm_data[2], \
                                          self.pos_data[0], self.pos_data[1], self.pos_data[2]),
                            font=('Digital-7', 11),
                            anchor=tk.W,
                            background="#FFE08C")
        self.label.pack(side=tk.RIGHT, fill=tk.Y)
        self.label.after(1, self.update)

        # Canvas
        self.fig = Figure(figsize=(10,10*CFG.MAP_WIDTH/CFG.MAP_HEIGHT))
        self.a = self.fig.add_subplot(111)
        im = plt.imread(CFG.MAP_PATH)

        self.a.imshow(im, extent=[0,CFG.MAP_WIDTH,0,CFG.MAP_HEIGHT])
        self.a.set_xlim(0, CFG.MAP_WIDTH)
        self.a.set_ylim(0, CFG.MAP_HEIGHT)
        self.a.set_title("GPS 2D Plot", fontsize=16)
        self.a.set_xlabel("x", fontsize=10)
        self.a.set_ylabel("y", fontsize=10)
        self.ann_list = []

        self.canvas = FigureCanvasTkAgg(self.fig, master=topFrame)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)

        self.mode = mode
        if self.mode == 'wp_plot':
            # Button
            self.button1 = tk.Button(master=self, text="Set Waypoint I", command=lambda:self.handle_click(1))
            self.button2 = tk.Button(master=self, text="Set Waypoint II", command=lambda:self.handle_click(2))
            self.button3 = tk.Button(master=self, text="Set Waypoint III", command=lambda:self.handle_click(3))
            self.button4 = tk.Button(master=self, text="Set Waypoint IV", command=lambda:self.handle_click(4))
            self.button5 = tk.Button(master=self, text="Set Waypoint V", command=lambda:self.handle_click(5))

            self.button5.pack(side=tk.BOTTOM, fill=tk.X)
            self.button4.pack(side=tk.BOTTOM, fill=tk.X)
            self.button3.pack(side=tk.BOTTOM, fill=tk.X)
            self.button2.pack(side=tk.BOTTOM, fill=tk.X)
            self.button1.pack(side=tk.BOTTOM, fill=tk.X)
        elif self.mode == 'wp_read':
            csv_file = open(CFG.CSV_PATH)
            csvreader = csv.reader(csv_file)

            header = []
            header = next(csvreader) # latitude,longitude,altitude,x,y,z

            for idx, row in enumerate(csvreader):
                csv_local_x = float(row[3]) - CFG.MAP_ORIGIN_UTM[0]
                csv_local_y = float(row[4]) - CFG.MAP_ORIGIN_UTM[1]
                csv_local_z = float(row[5]) - CFG.MAP_ORIGIN_UTM[2]
                self.a.scatter(csv_local_x, csv_local_y, color="yellow")
                self.a.annotate("WP_{}".format(idx+1), (csv_local_x,csv_local_y), color="white", fontsize=8)

        
    def handle_click(self, idx):
        self.waypoints[idx-1] = self.pos_data
        
        for idx, wp in enumerate(self.waypoints):
            print("WP_{}: {}".format(idx+1, wp))

    def callback(self, data_lla, data_utm):

        self.lla_data = [round(data_lla.latitude, self.round_decimal_points), round(data_lla.longitude, self.round_decimal_points), round(data_lla.altitude, self.round_decimal_points)]
        self.utm_data = [round(data_utm.pose.position.x, self.round_decimal_points), round(data_utm.pose.position.y, self.round_decimal_points), round(data_utm.pose.position.z, self.round_decimal_points)]

        self.pos_data_prev = self.pos_data
        self.pos_data = np.around(np.subtract(self.utm_data, self.pond_utm), decimals=self.round_decimal_points)

        self.local_x_arr = np.append(self.local_x_arr, self.pos_data[0])
        self.local_y_arr = np.append(self.local_y_arr, self.pos_data[1])

    def update(self):
        self.label.configure(text="Latitude: {}\nLongitude: {}\nAltitude: {}\n\nUTM X: {}\nUTM Y: {}\nUTM Z: {}\n\nLocal X: {}\nLocal Y: {}\nLocal Z: {}" \
                                  .format(self.lla_data[0], self.lla_data[1], self.lla_data[2], \
                                          self.utm_data[0], self.utm_data[1], self.utm_data[2], \
                                          self.pos_data[0], self.pos_data[1], self.pos_data[2]))        
        
        # Show Path Accumulation
        self.a.scatter(self.local_x_arr, self.local_y_arr, color="green")

        if self.mode == 'wp_plot':
            # Show Waypoints (270 milliseconds delay)
            if len(self.ann_list) > 0:
                for idx, ann in enumerate(self.ann_list):
                    ann.remove()
                self.ann_list = []
            for idx, wp in enumerate(self.waypoints):
                if np.sum(wp) != 0:
                    self.a.scatter(wp[0], wp[1], color="yellow")
                    ann = self.a.annotate("WP_{}".format(idx+1), (wp[0],wp[1]), color="white")
                    self.ann_list.append(ann)

        # Current Relative Position (m)
        self.a.scatter(self.pos_data[0], self.pos_data[1], color="red")

        self.canvas.draw()
        self.label.after(1, self.update)

if __name__ == '__main__':
    
    rospy.init_node('heron_gui_node')

    argumentList = sys.argv[1:]
    options = "hm:"
    long_options = ["Help", "Mode"]

    try:
        arguments, values = getopt.getopt(argumentList, options, long_options)
        for currentArgument, currentValue in arguments:
            if currentArgument in ("-h", "--Help"):
                print("-h/--Help: To see available arguments.\n-m/--Mode: 'wp_plot' & 'wp_read'")
            elif currentArgument in ("-m", "--Mode"):
                print("Current Mode is set to", currentValue)
                window = Heron_GUI(currentValue)
                window.mainloop()

    except getopt.error as err:
        print(str(err))

    