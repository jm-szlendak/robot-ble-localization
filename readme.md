# Radio Beacons robot localization system

Software to deal with localization using Bluetooth Low Energy beacons RSSI (radio signal strength)
## Usage
1. Turn on Bluetooth
2. Run `roslaunch beacon_listener beacon_listener.launch`
3. If encountered problems, refer to `beacon_listener` readme
4. Run `roslaunch rssi2distance rssi2distance.launch`, providing config file in `--map` argument (refer to `rssi2distance/config/default_map.yaml` for syntax)
5. Run `rosrun trilateration trilateration.py` 

For now, pose is published to `/beacon_localization/bl_pose` topic and to `bl_pose` TF frame


### Troubleshooting
- to use PyCharm, add workspace 'setup.bash' sourcing to .bashrc, then start PyCharm with:

```
$ bash -i -c "/opt/pycharm-community/bin/pycharm.sh"
```
- when ROS does not see any nodes in package, just add executable attribute to node files : `$ chmod +x trilateration/scr/trilateration.py` 