# USV_GPS_GUI: GPS Based USV Map Plotting GUI
This repository is set for the testing at KAIST Duck Pond, Daejeon, South-Korea.<br>
In order to customize this repository for any other region, please follow the last instruction <b>3. CUSTOMIZE FROM SCRATCH</b>

# 1. HOW TO RUN
1. While GPS Data is being published by the data with topic name /ublox/fix (sensor_msgs::NavSatFix), run below command in <b>Shell #1</b>
```
$ roslaunch heron_lla2utm coordinate_convertion.launch
```
2. Run one of the below commands for different results in <b>Shell #2</b><br>

<b>(Help): </b>To see available arguments
```
$ rosrun heron_gui main_gui.py -h
```
<b>(Read):</b> Read and plot the waypoints from <b>./heron_gui/src/csv_files/<filename>.csv</b>
```
$ rosrun heron_gui main_gui.py -m wp_read # <filename> is set to 'square' as default, follow instruction below to customize
```
<b>(Export):</b> Use GUI to select waypoints and export into <b>./heron_gui/src/csv_files/<new_filename>.csv</b>
```
$ rosrun heron_gui main_gui.py -m wp_plot # <new_filename> is set to 'export_data' as default, follow instruction below to customize
```

# 2. CUSTOMIZE
## case 1: Change GPS-streaming ROSTopic name
## case 2: Change map file
## case 3: Change *.csv file
## case 4: Change GUI plot color
  
