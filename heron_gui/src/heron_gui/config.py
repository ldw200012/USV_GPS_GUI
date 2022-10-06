#!/usr/bin/python

import os
root_dir = os.path.dirname(__file__)

MAP_PATH = os.path.join(root_dir, "../images/map_satel.png")    # map_common.png | map_contour.png | map_satel.png

MAP_ORIGIN_LLA = [36.3672449, 127.3619089, 0.0]     # LLA value of (0, 0) position on the map image
MAP_ORIGIN_UTM = [353044.30, 4025928.39, 0.0]       # LLA value transformed to UTM via https://www.latlong.net/lat-long-utm.html
MAP_WIDTH = 126     # 지도 실제 너비 (m)
MAP_HEIGHT = 181    # 지도 실제 높이 (m)
_PRECISION = 7      # lla, utm decimal precision

# Waypoint CSV File Path
CSV_PATH = os.path.join(root_dir, "../csv_files/square.csv")                # CSV file to read in '-m wp_read' mode
EXPORT_CSV_PATH = os.path.join(root_dir, "../csv_files/export_test.csv")    # CSV file to export from '-m wp_plot' mode
MAX_WP2PLOT = 5                                                             # number of waypoints to export from '-m wp_plot' mode
                                                                            # Change Line 83 of main_gui.py as well (add more Buttons)

# Subscriber Name
GPS_SUBSCRIBER = "/ublox_gps/fix"   # ROSTOPIC sent from GPS sensor
UTM_SUBSCRIBER = "/Heron_UTMPose"   # ROSTOPIC sent from lla2utm roslaunch

# COLOR PALETTE
WAYPOINT_COLOR = "black"
CURRENT_POS_COLOR = "red"
PATH_TRACK_COLOR = "blue"