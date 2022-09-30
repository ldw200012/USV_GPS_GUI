# USV_GPS_GUI: GPS Based USV Map Plotting GUI
This repository is set for the testing at KAIST Duck Pond, Daejeon, South-Korea.<br>
In order to customize this repository for any other region, please follow the last instruction <b>3. CUSTOMIZE FROM SCRATCH</b>

# 1. HOW TO RUN
1. While GPS Data is being published by the data with topic name /ublox/fix (sensor_msgs::NavSatFix), run below command in <b>Shell #1</b>
```
$ roslaunch heron_lla2utm coordinate_convertion.launch
```
2. Run one of the below commands for different results in <b>Shell #2</b><br>

<b>(Help): To see available arguments</b>
```
$ rosrun heron_gui main_gui.py -h
```
<b>(Read):</b> Read and plot the waypoints from <b>USV_GPS_GUI/heron_gui/src/csv_files/<filename>.csv</b>
```
$ rosrun heron_gui main_gui.py -m wp_read # <filename> is set to 'square' as default, follow instruction below to customize
```
<b>(Export):</b> Use GUI to select waypoints and export into <b>USV_GPS_GUI/heron_gui/src/csv_files/<new_filename>.csv</b>
```
$ rosrun heron_gui main_gui.py -m wp_plot # <new_filename> is set to 'export_data' as default, follow instruction below to customize
```

# 1. CUSTOMIZATION
Open <b>USV_GPS_GUI/heron_gui/src/heron_gui/config.py</b> with any code editor.
1. Choose the map type (Simple / Satellite / Contour view) by comment/uncomment.
```
MAP_PATH = os.path.join(root_dir, "../images/map_common.png")
# MAP_PATH = os.path.join(root_dir, "../images/map_satel.png")
# MAP_PATH = os.path.join(root_dir, "../images/map_contour.png")
```
2. Write the correct filepath for <filename>.csv files.
```
# Waypoint CSV File Path
CSV_PATH = os.path.join(root_dir, "../csv_files/<filename>.csv") # Make sure to place <filename>.csv in USV_GPS_GUI/heron_gui/src/csv_files/
EXPORT_CSV_PATH = os.path.join(root_dir, "../csv_files/<new_filename>.csv")
```
3. Check if the name of the rostopic publishing GPS data is written correctly. If not, change the name.
```
# Subscriber Name
GPS_SUBSCRIBER = "/ublox_fix/gps"
```
4. Just in case you want to change the color of the plot.
```
# COLOR PALETTE
WAYPOINT_COLOR = "black"
CURRENT_POS_COLOR = "red"
PATH_TRACK_COLOR = "blue"
```
  
