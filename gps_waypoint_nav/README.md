# gps_waypoint_nav

Minimal ROS 2 GPS waypoint follower for an outdoor ground robot.

Inputs:

- `handsfree/rtk/gnss` (`sensor_msgs/NavSatFix`)
- `handsfree/rtk/cog` (`std_msgs/Float64`, degrees, optional but recommended)

Outputs:

- `/cmd_vel` (`geometry_msgs/Twist`)
- `~/status` (`std_msgs/String`)
- `~/target_fix` (`sensor_msgs/NavSatFix`)
- `~/waypoint_path` (`nav_msgs/Path`)
- `~/track_path` (`nav_msgs/Path`)

Start disabled for safety:

```bash
ros2 launch gps_waypoint_nav waypoint_follower.launch.py
```

Clear old waypoints before recording a new route:

```bash
cd /home/whr/cc_ws/tros_ws
source install/setup.bash
ros2 run gps_waypoint_nav clear_waypoints
```

Capture the current GNSS position into the source waypoint file:

```bash
cd /home/whr/cc_ws/tros_ws
source install/setup.bash
ros2 run gps_waypoint_nav capture_waypoint --ros-args -p name:=p1
```

The launch files default to the source waypoint file when the workspace source tree is present, so newly captured points are used immediately.

Hand-carried dry run: waypoint switching is enabled, but all velocity outputs are forced to zero.

```bash
ros2 launch gps_waypoint_nav waypoint_dry_run.launch.py
ros2 topic echo /gps_waypoint_follower/status
```

Start enabled after filling `config/waypoints.yaml`:

```bash
ros2 launch gps_waypoint_nav waypoint_follower.launch.py \
  enabled:=true
```

Combined RTK + follower debug launch:

```bash
ros2 launch gps_waypoint_nav gps_nav_debug.launch.py port:=/dev/HFRobotRTK
```

The first received GNSS fix becomes the local ENU origin used for RViz debug paths.
