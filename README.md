# ROS2 LiDAR Threshold Detector

This package is build on **Ubuntu 24.04** and **ROS2 Jazzy**.
A ROS2 package that subscribes to LiDAR (`/scan`) data, velocity (`/cmd_vel`). Detects obstacles inside a **threshold circle** (safety radius) and logs **Linear and Angular velocities**.  
If any object enters this safety circle, the node logs the obstacle’s **distance** and **angle**. It also stops robot form moving further. 

---

## Features
- Subscribes to **LaserScan** data from LiDAR.
- Subcribes to **/cmd_vel** for velocity logging. 
- Configurable **safety circle radius**.
- Logs all detected obstacles inside the circle with distance and angle.
- Logs linear and angular velocity. Also distance from goal.
- If obstacle is detected within threshold circle, it stops mobile robot. 
- Simple and lightweight ROS2 node.

---
## Logic:

#### Obstacle detection 
  LiDar rotates and send out light. When the value is not "INF", that means, object is there.
  Now, form same object light will bounce multiple times, which will result in multiple object detection alert. To counter this condition is applied, 
      if distance b/w consecutive points < threshold distance and angle b/w consecutive points < threshold angle, then it is same object. 
      else it will save the cluster and will create a new one. 
      


      To log the object distance and angle, average distance and angle is calcuated form the save cluster. 

      ![Object detection](/screenshots/gazebo_visulation.png)

#### Safe threshold
  **Safe radius** is inialized and from the same logic above, now **avg_dist < safe_radius**. It will log **Danger**, print avg_dist and avg_ang. 

  ![Object breaching threshold 1.1](/screenshots/safety_radius.png)
  ![Object breaching threshold 1.1](/screenshots/warning.png)


#### Velocity 
  Robot subcribe to **/cmd_vel** for velocity to log linear and angular velocity. 
  ![Logging velocity](/screenshots/Velocity.png)

  ![Command to stop robot when obstacle is near](/screenshots/safety_radius.png)


#### Obstacle avoidance 
  Taking logic from safety radius. 
    if avg_dist < safety_radis , Command is send by (`/Bool`) to stop the robot. 
## Flow-Chart:


## Installation

Clone the package into your ROS2 workspace `src` folder:

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ros2-lidar-detector.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```


---

## Usage

Run your simulation (e.g., Gazebo with a robot publishing `/scan`) and then:

```bash
ros2 launch mobile_robot full
```

In another terminal for navigation, 
```bash
source install/setup.bash

ros2 run mobile_robot point
```
This launches log linear and angular velocity, position, number of waypoints crossed 
---

## Parameters

- **threshold_radius** (float, default: `1.0`)  
  The radius of the safety circle in meters. Any obstacle inside this radius is reported.
- **angle_threshold** (float, default: `5.0`) 
  The max. angle allowed to change
- **distance_threshold** (float, default: `0.3`)
  The max. distance allowed to change
- **linear_speed** (float, default: `0.2`)
  Linear velocity of robot in m/sec (it can also be change)
- **angular_speed** (float, default: `0.5`)
  Speed at which robot turn in rad/sec
- **distance_tolorance** (float, default: `0.1`)
  When the value is less than distance, then waypoint reached

---

## Example Output

```text
    Object(s) detected inside 1.00 m:
  Object 1: Distance = 0.85 m, Angle = -30.0°
  Object 2: Distance = 0.90 m, Angle = 15.0°
```
![Object breaching threshold 1.1](/screenshots/object_detection.png)
If no objects are detected:

```text
No objects inside threshold circle.
```

---

## Demo Vedio
  [Demo vedio] ()
## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
