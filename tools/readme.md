### Tools
##### Toolset for beacon data analisys.
Those scipts are not ROS nodes! They are made for standalone use.

#### plot_profile.py
A tool to plot RSSI vs. distance profile. Usage:
```
$ python plot_profile.py [input file name]
```
Input file is expected to have 3 columns in following order: 

 Distance | Average RSSI | Standard Deviation
 
#### plot_histogram.py
A tool to plot RSSI histogram.
```
$ python plot_histogram.py [input file name]
```
Input file is expected to have 1 column containing measures. 
