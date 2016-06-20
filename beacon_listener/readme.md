# Beacon Listener

ROS node designed to listen to beacons advertising.

##Troubleshooting
1. How to scan without root permissions
```
$ sudo apt-get install libcap2-bin
$ sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/local/lib/python2.7/dist-packages/bluepy/bluepy-helper
```