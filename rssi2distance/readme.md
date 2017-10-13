# RSSI2Distance

This package contains node for listening to beacons RSSI values, filtering them and converting RSSI to distance. 
Distance conversion in performed using parametrized model. 

The node serves family of services: `/beacon_localization/distances/*`, where '*' denotes filtering mode
Currently supported filters are:
* `recent` - only recent value of RSSI for given beacon
* `moving_avg` - moving average of last N beacon RSSI values.
* `probabilistic` - simple FIR filter.


