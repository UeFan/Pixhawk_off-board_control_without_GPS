# offb_velocity_control_without_GPS

Offboard control using [ROS](http://www.ros.org) and [MAVROS](https://github.com/mavlink/mavros) for [PX4](https://github.com/PX4/Firmware).

The initial implementation is taken from the [MAVROS offboard control example](http://dev.px4.io/ros-mavros-offboard.html).

To control the drone in a indoor environment without GPS, we pushlish the topic that controls the velocity of the drone.
We also add a joystick/keyboard interface to simulate the real drone controller.


This project is created for our drone team in SDU, we took part in RoboWork Competition for several years.
!!! Please note that due to the accumulated error, the drone can easily drift around, which is hard to control.

## Usage

Outline:
First, create a new catkin_workspace or use a exist catkin_ws.
Then, copy px4_velocity_control_with_no_GPS folder in to "catkin_ws/src", as a package.
Make catkin workspace.
Finally, run with roslaunch.

### Dependencies

- [ROS](http://www.ros.org)
- [MAVROS](https://github.com/mavlink/mavros)
- [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Building

```
cd catkin_ws
catkin_make
```

### Running

Start MAVROS:

```
roslaunch mavros px4.launch fcu_url:="(your device)"
```

And run:

```
roslaunch offb offb.launch
```

