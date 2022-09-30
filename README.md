# USV_GPS_GUI
GPS Based USV Map Plotting GUI
This repository is set for the testing at KAIST Duck Pond, Daejeon, South-Korea.
In order to customize this repository for any other region, please follow the last instruction <b>3. CUSTOMIZE FROM SCRATCH</b>

# 1. CUSTMONIZATION
Open USV_GPS_GUI/heron_gui/src/heron_gui/config.py with any code editor.
* MAP_PATH: 

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
  
