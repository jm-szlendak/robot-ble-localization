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

