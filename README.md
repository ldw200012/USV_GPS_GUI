# USV_GPS_GUI
GPS Based USV Map Plotting GUI

# 1. HOW TO RUN

```
$ roslaunch heron_lla2utm coordinate_convertion.launch
```

```
$ rosrun heron_gui main_gui.py -<argument> <option>
```
  - <argument>: h (Help command to see available arguments) | m (Select one of the mode by entering <option> after)
  - <option>: wp_read (Read the existing file /heron_gui/src/csv_files/<filename>.csv) | wp_plot (Use GUI to plot and export the waypoints as *.csv file)
