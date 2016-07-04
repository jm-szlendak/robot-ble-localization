# Beacon Listener

ROS node designed to listen to beacons advertising.

##Advertising packet structure
|1  |2|3|4  |5|6|7|8|9|10|11|12|13|14|15|16|17|18|19 |20|21|22|23|24|25|26|27|28|29|30|31|
|:-:|-|-|:-:|-|-|-|-|-|- |- |- |- |- |- |- |- |- |:-:|- |- |- |- |- |- |- |- |- |- |- |- |
|flags|||name|||||||||||||||                  manufacturer-specific |||||||||||||
| |||RRG WUT #xx|||||||||||||||                  0xFFFF|Group ID||||Beacon ID||||||||||||


##Advertising filtering rules
We name beacons "RRG WUT #nn", but it's only for operator's convinience.
To filter out unwanted packets application checks 'manufacturer-specific field' with following rules:

 * Manufacturer ID is expected to be 0xFFFF
 * 2 first bytes of payload is 'beacon-set-ID'. This programmable parameter is used to differ sets of beacons
 * Rest of bytes are beacon-individual ID
 
##Troubleshooting
1. How to scan without root permissions
```
$ sudo apt-get install libcap2-bin
$ sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/local/lib/python2.7/dist-packages/bluepy/bluepy-helper
```

