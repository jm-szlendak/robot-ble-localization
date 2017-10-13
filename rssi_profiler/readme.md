## RSSI Profiler

#### Usage 
First, run profiler node to make profile service available:
```
$ rosrun rssi_profiler rssi_profiler.py
```
Then, you can measure RSSI:
```
$ rosrun rssi_profiler scribe.py [distance] [measurements count] [output file] [mode]
```

`distance` - Distance between receiver and beacon.

`measurements count` - Number of measurements scribe will ask for.

`output file` - Output file name. Data will be appended to this file.

`mode` - 'single' when you collect many measurements for one distances (e.g. for histogram plot), 'multi' for many measurements

#### Data file structure

##### Single mode
```
-48.0
-51.0
-51.0
-51.0
```

Column represents consecutive measurements.

##### Multi mode
```
0.0     -51.6   0.8
0.0     -53.2   0.979795897113
```

1st column - distance

2nd column - average RSSI

3rd column - standard deviation

#MAGICAL Scribe
When using robot with reliable localization like AMCL, you can collect profile magically. 
Run beacon_listener and rssi2distance (you need beacon map and dummy rssi-distance model, it can be dummy model)
Then:
```
$ rosrun rssi_profiler magical_scribe.py 
```
Magical scribe will query TFs from robot base-link to beacons and calculate measurement point. 
Run magical scribe with `--help` to get available arguments.