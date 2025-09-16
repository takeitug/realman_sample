# realman_sample

This repository provides sample code for **Cartesian velocity control** of the **Realman Robot** on **ROS Noetic**.  
It depends on the Realman official ROS driver **[`rm_robot`](https://github.com/realmanrobot/rm_robot)** .
Currently developed and tested with the **RM65_6F** robot arm from the RM series.

---

## Usage

### Velocity Control Node
```bash
roslaunch realman_sample twistclient
````

* Publish `geometry_msgs/Twist` messages to `/twist_cmd`.
  These are converted to joint positions by inverse kinematics and then published to `/rm_driver/JointPos`.
* Use `/twist_enable` (`std_msgs/Bool`) to toggle velocity control **on/off**.

---

## Sample Codes

### Joystick Control

```bash
roslaunch realman_sample teleop
```

Example of velocity control with a DualSense (PS5 controller).

* **Circle button (○)**: enable velocity control
* **Cross button (×)**: disable velocity control
* Hold **L1** and move the sticks to control the robot.

### Combined Joint Position and Velocity Control

```bash
rosrun realman_sample demo
```

Demonstration that combines **MoveJ joint position control** with **Cartesian velocity control**.
The robot first moves to a target joint configuration, then performs a constant velocity motion along the Z-axis, and finally stops.

---

## Parameters

### `tau_pred`

* Controls the compensation for the delay between the command and the real robot.
* Recommended range: **0.0 – 0.5**.
* Larger values reduce the apparent delay but increase the risk of overshoot.

---

## Notes

* Tested with **ROS Noetic** on **Ubuntu 20.04**.

```
```
