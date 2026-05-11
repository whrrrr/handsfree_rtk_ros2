# GPS RTK Navigation Stack

ROS 2 packages for RTK positioning and simple GPS waypoint navigation.

Packages:

- `handsfree_rtk`: HandsFree RTK receiver driver and launch files.
- `gps_waypoint_nav`: waypoint capture, dry-run route checking, and `/cmd_vel` waypoint follower.

Typical workflow:

```bash
cd /home/whr/cc_ws/tros_ws
source install/setup.bash

ros2 launch handsfree_rtk handsfree_rtk.launch.py port:=/dev/HFRobotRTK
ros2 run gps_waypoint_nav clear_waypoints
ros2 run gps_waypoint_nav capture_waypoint --ros-args -p name:=p1
ros2 launch gps_waypoint_nav waypoint_dry_run.launch.py
```

Use RTK Fixed (`GGA fix=4`) before recording or following waypoints.
