# USV_GPS_GUI
GPS Based USV Map Plotting GUI

# 1. HOW TO RUN

```
$ roslaunch heron_lla2utm coordinate_convertion.launch
```

```
$ rosrun heron_gui main_gui.py -<argument> <option>
```
  > $emsp; rosrun heron_gui main_gui.py -h # (Help command to see available arguments)
  
  > $emsp; rosrun heron_gui main_gui.py -m wp_read # (Read the existing file USV_GPS_GUI/heron_gui/src/csv_files/<filename>.csv)
  
  > $emsp; rosrun heron_gui main_gui.py -m wp_plot # (Use GUI to plot and export the waypoints as USV_GPS_GUI/heron_gui/src/csv_files/<filename>.csv)
  
