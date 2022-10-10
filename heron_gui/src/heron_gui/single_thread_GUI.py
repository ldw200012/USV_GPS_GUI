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

        self.lla_data = [0.0, 0.0, 0.0]
        self.utm_data = [0.0, 0.0, 0.0]

        self._scat = None
        self.pos_data_prev = [0.0, 0.0, 0.0]
        self.pos_data = [0.0, 0.0, 0.0]

        self.local_x_arr = np.array([])
        self.local_y_arr = np.array([])
        self.waypoints = np.zeros((CFG.MAX_WP2PLOT,6))

        # Subscriber for GPS(LLA)
        rospy.Subscriber(CFG.GPS_SUBSCRIBER, NavSatFix, self.callback)

        self.wp_points = []
        self.wp_annots = []
        self.create_gui(mode)

    # CREATE GUI (change 'config.py' for customization)
    def create_gui(self, mode):
        topFrame = tk.Frame(self, width=720, height=720)
        topFrame.pack_propagate(0)
        topFrame.pack(side=tk.TOP)
        
        # HEADER & POSITION VALUES to print (LLA & UTM)
        self.title = "Heron GPS Tracking & Waypoint Viewer GUI"
        self.label = ttk.Label(topFrame,
                            text="Latitude: {:.2f}\nLongitude: {:.2f}\nAltitude: {:.2f}\n\nUTM X: {:.2f}\nUTM Y: {:.2f}\nUTM Z: {:.2f}\n\nLocal X: {:.2f}\nLocal Y: {:.2f}\nLocal Z: {:.2f}" \
                                  .format(self.lla_data[0], self.lla_data[1], self.lla_data[2], \
                                          self.utm_data[0], self.utm_data[1], self.utm_data[2], \
                                          self.pos_data[0], self.pos_data[1], self.pos_data[2]),
                            font=('Digital-7', 11),
                            anchor=tk.W,
                            background="#FFE08C",
                            width=20)
        self.label.pack(side=tk.RIGHT, fill=tk.Y)
        self.label.after(1, self.update)

        # MAP PLOT IMAGE VIEW
        height = 10
        self.fig = Figure(figsize=(height,height*CFG.MAP_WIDTH/CFG.MAP_HEIGHT))
        self.a = self.fig.add_subplot(111)
        im = plt.imread(CFG.MAP_PATH)

        self.a.imshow(im, extent=[0,CFG.MAP_WIDTH,0,CFG.MAP_HEIGHT], alpha=0.7)
        self.a.set_xlim(0, CFG.MAP_WIDTH)
        self.a.set_ylim(0, CFG.MAP_HEIGHT)
        self.a.set_title("GPS 2D Plot", fontsize=16)
        self.a.set_xlabel("x", fontsize=10)
        self.a.set_ylabel("y", fontsize=10)

        self.canvas = FigureCanvasTkAgg(self.fig, master=topFrame)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.Y)

        # FOOTER depends on MODE (wp_plot, wp_read)
        self.mode = mode
        if self.mode == 'wp_plot':
            # Add Buttons
            self.button1 = tk.Button(master=self, text="Set Waypoint I", command=lambda:self.handle_click(1))
            self.button2 = tk.Button(master=self, text="Set Waypoint II", command=lambda:self.handle_click(2))
            self.button3 = tk.Button(master=self, text="Set Waypoint III", command=lambda:self.handle_click(3))
            self.button4 = tk.Button(master=self, text="Set Waypoint IV", command=lambda:self.handle_click(4))
            self.button5 = tk.Button(master=self, text="Set Waypoint V", command=lambda:self.handle_click(5))
            # self.button6 = tk.Button(master=self, text="Set Waypoint VI", command=lambda:self.handle_click(6))
            # ...
            self.btn_export = tk.Button(master=self, text="Export to CSV", command=lambda:self.csv_export_wp(CFG.EXPORT_CSV_PATH))

            self.btn_export.pack(side=tk.BOTTOM, fill=tk.X)
            # ...
            # self.button6.pack(side=tk.BOTTOM, fill=tk.X)
            self.button5.pack(side=tk.BOTTOM, fill=tk.X)
            self.button4.pack(side=tk.BOTTOM, fill=tk.X)
            self.button3.pack(side=tk.BOTTOM, fill=tk.X)
            self.button2.pack(side=tk.BOTTOM, fill=tk.X)
            self.button1.pack(side=tk.BOTTOM, fill=tk.X)

        elif self.mode == 'wp_read':
            csv_file = open(CFG.CSV_PATH)
            csvreader = csv.reader(csv_file)
            header = []
            header = next(csvreader) # ignore the first row of CSV ([latitude,longitude,altitude,x,y,z])

            for idx, row in enumerate(csvreader):
                # Find local position to plot (UTM - MAP's_ORIGIN_UTM)
                csv_local_x = float(row[3]) - CFG.MAP_ORIGIN_UTM[0]
                csv_local_y = float(row[4]) - CFG.MAP_ORIGIN_UTM[1]
                self.a.scatter(csv_local_x, csv_local_y, color=CFG.WAYPOINT_COLOR, marker='.', edgecolors='none')
                self.a.annotate("WP_{}".format(idx+1), (csv_local_x,csv_local_y), color="white", fontsize=8)

    # SAVE CURRENT POSE as a waypoint
    def handle_click(self, idx):
        self.waypoints[idx-1] = np.append(self.lla_data, self.utm_data)

        # Delete Waypoints drawn in previous frame
        for _ in self.wp_points:
            _.remove()
        for _ in self.wp_annots:
            _.remove()
        self.wp_points = []
        self.wp_annots = []

        # Draw new Waypoins 
        for idx, wp in enumerate(self.waypoints):
            if np.sum(wp) != 0:
                local_x = wp[3] - CFG.MAP_ORIGIN_UTM[0]
                local_y = wp[4] - CFG.MAP_ORIGIN_UTM[1]

                wp_point = self.a.scatter(local_x, local_y, color=CFG.WAYPOINT_COLOR, marker='.', edgecolors='none')
                wp_annot = self.a.annotate("WP_{}".format(idx+1), (local_x, local_y), color="white")

                self.wp_points.append(wp_point)
                self.wp_annots.append(wp_annot)

        
        # Print the current status of set waypoints in terminal (Optional)
        # for idx, wp in enumerate(self.waypoints):
        #     print("WP_{}: {}".format(idx+1, wp))

    # EXPORT CSV FILE of set waypoints from GUI
    def csv_export_wp(self, path):
        f = open(path, 'w')
        writer = csv.writer(f)
        writer.writerow(["latitude","longitude","altitude","x","y","z"])
        for wp in self.waypoints:
            if np.sum(wp) != 0:
                writer.writerow(wp)
        f.close()
        print("WAYPOINTS are saved into CSV file")

    def callback(self, data_lla):
        # Set Current Position as LLA & UTM & Local_XYZ(the point position on map image (m))
        self.lla_data = [data_lla.latitude, data_lla.longitude, data_lla.altitude]
        self.lla_data = np.around(self.lla_data, decimals=CFG._PRECISION)


        self.utm_data = self.lla2utm(self.lla_data)
        print(self.utm_data)
        self.utm_data = np.around(self.utm_data, decimals=CFG._PRECISION)

        self.pos_data_prev = self.pos_data
        self.pos_data = np.around(np.subtract(self.utm_data, CFG.MAP_ORIGIN_UTM), decimals=CFG._PRECISION)

        # Accumulate pose to show the Path on the map
        self.local_x_arr = np.append(self.local_x_arr, self.pos_data[0])
        self.local_y_arr = np.append(self.local_y_arr, self.pos_data[1])

    def lla2utm(self, lla_data):
        dLatitude = lla_data[0]
        dLongitude = lla_data[1]

        dLat = dLatitude * np.pi/180
        dLon = dLongitude * np.pi/180

        lon0_f = np.floor(dLongitude/6)*6+3
        lon0 = lon0_f*np.pi/180
        k0 = 0.9996
        FE = 500000
        FN = (dLatitude < 0)*10000000

        Wa = 6378137
        We = 0.081819190842965
        WN = Wa/np.sqrt( 1 - np.power(We,2)*np.power(np.sin(dLat),2))
        WT = np.power(np.tan(dLat), 2)
        WC = (np.power(We,2)/(1-np.power(We,2)))*np.power(np.cos(dLat),2)
        WLA = (dLon - lon0)*np.cos(dLat)
        WM = (Wa*((1 - np.power(We,2)/4 - 3*np.power(We,4)/64 - 5*np.power(We,6)/256)*dLat - (3*np.power(We,2)/8 + 3*np.power(We,4)/32 + 45*np.power(We,6)/1024)*np.sin(2*dLat) + (15*np.power(We,4)/256 + 45*np.power(We,6)/1024)*np.sin(4*dLat) - (35*np.power(We,6)/3072)*np.sin(6*dLat)) )

        Weps = 0.006739496742333
        # Easting
        m_dUTM_X = (FE + k0*WN*(WLA + (1 - WT + WC)*np.power(WLA,3)/6	+ (5 - 18*WT + np.power(WT,2) + 72*WC - 58*Weps)*pow(WLA,5)/120))
        # Northing
        m_dUTM_Y = (FN + k0*WM + k0*WN*np.tan(dLat)*(np.power(WLA,2)/2 + (5 - WT + 9*WC + 4*np.power(WC,2))*np.power(WLA,4)/24 + (61 - 58*WT + np.power(WT,2) + 600*WC - 330*Weps)*np.power(WLA,6)/720))
        # Zone
        m_nUTM_Zone = np.int(np.floor(lon0_f/6)+31)

        print("Transformation Done")
        return [m_dUTM_X, m_dUTM_Y, 0]

    def update(self):
        # UPDATE POSITION VALUES to print in GUI(LLA & UTM)
        self.label.configure(text="Latitude: {:.2f}\nLongitude: {:.2f}\nAltitude: {:.2f}\n\nUTM X: {:.2f}\nUTM Y: {:.2f}\nUTM Z: {:.2f}\n\nLocal X: {:.2f}\nLocal Y: {:.2f}\nLocal Z: {:.2f}" \
                                  .format(self.lla_data[0], self.lla_data[1], self.lla_data[2], \
                                          self.utm_data[0], self.utm_data[1], self.utm_data[2], \
                                          self.pos_data[0], self.pos_data[1], self.pos_data[2]))        
        
        if self.mode == 'wp_read':
            # Show Path Accumulation
            self.a.scatter(self.local_x_arr, self.local_y_arr, color=CFG.PATH_TRACK_COLOR, marker='.', edgecolors='none')

        # Current Position on map (m)
        if self._scat is not None:
            self._scat.remove()
        self._scat = self.a.scatter(self.pos_data[0], self.pos_data[1], color=CFG.CURRENT_POS_COLOR, marker='x', edgecolors='none')

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

    