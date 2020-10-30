# harveynet-adapter

A ROS package that connects a ROS TurtleBot to the HarveyNet.

This adapter package (node) will authorize the TurtleBot at the HarveyNet as `machine1`, owned by user `alice@email.com` by default (the machine ID is *hard-coded*).

It is assumed that you have all the ROS infrastructure installed and configured on your computer, including the TurtleBot. Also the stable version of Node.js is required.

## Installation and running

1. Clone the package repository under `<CATKIN_WS_DIR>/src`.

```bash
$ cd <CATKIN_WS_DIR>/src
$ rosdep install --from-paths src -i -y
$ git clone <THIS_REPO_URL>
$ cd harveynet_adapter
```

2. Install dependencies:

```bash
$ npm install
```

3. It may be necessary to run `catkin_make` (from corresponding location).

4. (Assuming you are still in the directory from step 1) make the `adapter_node.js` executable:

```bash
$ chmod +x ./scripts/adapter_node.js
```

5. Source the setup file:

```bash
$ . ~/catkin_ws/devel/setup.bash
```

6. Launch the TurtleBot Gazebo:

turtlebot 3:
```bash
$ TURTLEBOT3_MODEL=waffle roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

7. In a new terminal, run the adapter node:

```
$ rosrun harveynet_adapter adapter_node.js
```
Now the TurtleBot machine is connected to the HarveyNet.

8. Open the HarveyNet Control Panel in your web browser: https://harveynet-control-panel.herokuapp.com

9. Login as user `alice@email.com`, select machine `machine1`.

10. Now you can control the Turtlebot from the HarveyNet.

11. (Optionally) Kill the adapter node process to see the machine going offline at the Control Panel.


## Setting machine ID (auth)

```
$ MACHINE_ID=machine4 rosrun harveynet_adapter adapter_node.js
```

If `MACHINE_ID` is not set, it defaults to `machine1`.


## Switching to TurtleBot 2

```
$ TURTLEBOT2=true rosrun harveynet_adapter adapter_node.js
```

If the `TURTLEBOT2` variable is omitted, the adapter assumes that TurtleBot 3 is being used.

# Debugging

And error similar to the following happened when a subscriber and a publisher sourced two different versions of a message:

[ERROR] [1604055855.947] (ros.rosnodejs): Unable to validate subscriber connection header {"callerid":"/rostopic_11509_1604055761698","md5sum":"0d075c0ce6543ae3c7e94979a356b760","message_definition":"std_msgs/Header header\nbool joystick_mode\nbool engine_on\nbool engine_off\nfloat32 left_track_speed\nfloat32 right_track_speed\nint16 track_speed_max_increment\nint8 front_belt_rpm_increment\nint8 back_belt_rpm_increment\nint8 dammer_height\nbool hmi_connected\nint8 pallet_height\nint8 pallet_tilt\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n","tcp_nodelay":"0","topic":"/harvey_controller/hmi_controller","type":"harvey_can/harvey_joy_msg"}

The solution is to run catkin_make again in the workspace that holds the messages,
and after that sourcing it again for all nodes that use that message.
