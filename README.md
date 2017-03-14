Steps to run:

1. Put package in a catkin workspace (say `~/catkin_ws`)
2. Do `catkin_make` in `~/catkin_ws` 
3. `source /devel/setup.bash`
3. Plug-in FemtoBeacon dongle, assumes the port to be /dev/ttyACM0.

  Baudrate is 115200 by default, has to match the FemtoBeacon's [Arduino Software baudrate](https://github.com/allenyin/ArduinoCore-atsamd21e18a/blob/master/libraries/FemtoBeacon_Rf/FemtoBeacon_Rf_MESH_IMU_Dongle/FemtoBeacon_Rf_MESH_IMU_Dongle.ino).

4. `roscore`
5. `rosrun femtobeacon_ros femtobeacon_node.py`


