# Beacon Listener

ROS node designed to listen to beacons advertising.

##Installation

First, install bluepy (hope it will be automated in future releases):
```
    $ sudo apt-get install python-pip libglib2.0-dev
    $ sudo pip install bluepy
```

If you want to run this package without root permissions (probably you do):
```
$ sudo apt-get install libcap2-bin
$ sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/local/lib/python2.7/dist-packages/bluepy/bluepy-helper
```


##Usage:
```
$ rosrun beacon_listener beacon_listener.py
```

### Topics

`/beacon_localization/location_tag` - message type: beacon_msgs/LocationTag

##Advertising packet structure
In square brackets field sizes are given. They are equal 1 (length byte) +  X (actual payload size) 
```
| flags [1+2 B] | name [1+12 B] | payload [1+14 B] |

```

Payload:

```
| 0xFFFF [2B] | group id [4B] | beacon id [6B] | padding [1B]
```

`0xFFFF` is always constant, it is Bluetooth beacon manufacturer code. 

Group ID is for identifying beacon sets.

Beacon ID is for identifying individual beacons. It is equal to beacon MAC address.
 
##Advertising filtering rules
We name beacons "RRG WUT #nn", but it's only for operator's convinience.
To filter out unwanted packets application checks 'manufacturer-specific field' with following rules:

 * Manufacturer ID is expected to be 0xFFFF
 * Group ID match. Filter with pass only beacons from given group.
 
 Beacon ID is not checked in filter, although a filter which filters out specific beacons can be implemented. 
 
##Troubleshooting
1. How to scan without root permissions
```
$ sudo apt-get install libcap2-bin
$ sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/local/lib/python2.7/dist-packages/bluepy/bluepy-helper
```

If this does not work, try casting a spell:
```
$ sudo chmod 4751 /usr/local/lib/python2.7/dist-packages/bluepy/bluepy-helper

```

