#!/usr/bin/python

import os
root_dir = os.path.dirname(__file__)

# Reference Map Image Path
# 오리연못 가로 126m, 세로 179m (126/179 = 0.7039)
# 연못사진 가로 523px, 세로 743px (523/743 = 0.7039)
# Plot 크기 가로 7in, 세로 9.944in (9.944/7 =0.7039)
MAP_PATH = os.path.join(root_dir, "../images/pond_map.png")
MAP_ORIGIN_LLA = [36.3672449, 127.3619089, 0.0]
MAP_ORIGIN_UTM = [353044.30, 4025928.39, 0.0]
MAP_WIDTH = 126
MAP_HEIGHT = 179

# Waypoint CSV File Path
CSV_PATH = os.path.join(root_dir, "../csv_files/square.csv")

# Subscriber Name
GPS_SUBSCRIBER = "/ublox/fix"
UTM_SUBSCRIBER = "/Heron_UTMPose"