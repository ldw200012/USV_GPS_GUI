# USV_GPS_GUI: GPS Based USV Map Plotting GUI
This repository is set for the testing at KAIST Duck Pond, Daejeon, South-Korea.<br>
In order to customize this repository for any other region, please follow the last instruction <b>3. CUSTOMIZE FROM SCRATCH</b>

# 1. CUSTOMIZATION
Open USV_GPS_GUI/heron_gui/src/heron_gui/config.py with any code editor.
<br><br>
1. Choose the map type (Simple / Satellite / Contour view) by comment/uncomment.
```
MAP_PATH = os.path.join(root_dir, "../images/map_common.png")
# MAP_PATH = os.path.join(root_dir, "../images/map_satel.png")
# MAP_PATH = os.path.join(root_dir, "../images/map_contour.png")
```
<br><br>
2. Write the correct filepath for <filename>.csv files.
```
# Waypoint CSV File Path
CSV_PATH = os.path.join(root_dir, "../csv_files/<filename>.csv") # Make sure to place <filename>.csv in USV_GPS_GUI/heron_gui/src/csv_files/
EXPORT_CSV_PATH = os.path.join(root_dir, "../csv_files/<new_filename>.csv")
```
<br><br>
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

# 2. HOW TO RUN
```
$ roslaunch heron_lla2utm coordinate_convertion.launch
```

```
$ rosrun heron_gui main_gui.py -<argument> <option>
```
  > rosrun heron_gui main_gui.py -h # (Help command to see available arguments)
  
  > rosrun heron_gui main_gui.py -m wp_read # (Read the existing file USV_GPS_GUI/heron_gui/src/csv_files/<filename>.csv)
  
  > rosrun heron_gui main_gui.py -m wp_plot # (Use GUI to plot and export the waypoints as USV_GPS_GUI/heron_gui/src/csv_files/<filename>.csv)
  
