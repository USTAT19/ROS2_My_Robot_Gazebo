# ROS2 My Robot Gazebo

This repository contains a **ROS 2 workspace** for simulating and visualizing a custom robot using  
**ROS 2 Jazzy Jalisco**, **Gazebo Harmonic**, and **RViz2** on **Ubuntu 24.04**.

The robot includes a **camera sensor**, which can be visualized directly in **RViz2**.

It is intended for learning, experimentation, and robotics competitions.

---

## 📂 Repository Structure

ROS2_My_Robot_Gazebo/
│
├── src/
│ ├── my_robot_description/ # URDF / XACRO robot model
│ └── my_robot_bringup/ # Launch files for Gazebo + RViz2
│
├── .gitignore
├── README.md
├── LICENSE


---

## 🚀 System Requirements

- **Ubuntu 24.04 (Noble Numbat)**
- **ROS 2 Jazzy Jalisco**
- **Gazebo Harmonic**
- **RViz2**
- **colcon**
- **xacro**

---

## 🛠️ Installation & Setup

### 1️⃣ Install ROS 2 Jazzy
Follow the official installation guide:  
https://docs.ros.org/en/jazzy/Installation.html

### 2️⃣ Install Gazebo Harmonic
```bash
sudo apt install gz-harmonic
```
3️⃣ Install build tools
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
```
Initialize rosdep (once):
```bash
sudo rosdep init
rosdep update
```
🔧 Build Instructions:

1️⃣ Clone the repository

```bash
git clone https://github.com/USTAT19/ROS2_My_Robot_Gazebo.git
cd ROS2_My_Robot_Gazebo
```
2️⃣ Install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```
3️⃣ Build the workspace

```bash
colcon build
```
4️⃣ Source the workspace

```bash
source install/setup.bash
```
▶️ Run the Simulation
Launch the robot in Gazebo Harmonic and RViz2:

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
👁️ RViz2 Visualization
In RViz2, you can visualize:
Robot model (RobotModel)
TF frames (TF)
Camera image stream (Image)

📷 Camera Sensor
The robot includes a camera sensor defined in the URDF/XACRO file.
Typical camera topic:

/camera/image_raw
#View camera feed in RViz2
Open RViz2
Click Add → Camera
Set the topic to /camera/image_raw

🎮 Robot Control (Teleoperation)
The mobile base of the robot is controlled using keyboard teleoperation via
the teleop_twist_keyboard package.

🔧 Teleoperation Method
Package: teleop_twist_keyboard
Message type: geometry_msgs/Twist
Command topic:cmd_vel

The velocity commands published on /cmd_vel are consumed by the
Gazebo Harmonic DiffDrive plugin (gz::sim::systems::DiffDrive) to drive the robot.

▶️ Run Teleop Node
In a new terminal (after sourcing the workspace):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

🦾 Arm Joint Control via ROS 2 Topic

In addition to keyboard teleoperation for the mobile base, arm joints are controlled by publishing position commands directly to ROS 2 topics.
The Gazebo Harmonic JointPositionController plugin listens to command topics of type:

Message type: std_msgs/msg/Float64

Control mode: Position control

▶️ Publishing Joint Position Commands

You can manually command a joint position using:
```bash
ros2 topic pub -1 /joint0/cmd_pos std_msgs/msg/Float64 "{data: 0.8}"
```

🔧 Explanation

/joint0/cmd_pos → Command topic for the joint
0.8 → Target joint position (in radians)
-1 → Publishes the message once

This command sends a desired position to the joint, which is then executed by the
gz::sim::systems::JointPositionController plugin in Gazebo Harmonic.

📌 Notes
Each controlled joint has its own command topic
Topic names depend on how the joint controller is configured in the URDF/XACRO
This method is useful for:
Testing arm motion
Debugging controllers
Simple manipulation experiments (without MoveIt)

🔌 Gazebo Harmonic Plugins Used
This project uses Gazebo Harmonic system plugins for robot motion control
and joint state publishing.

🦾 Joint Position Control (Arm)
Plugin: gz::sim::systems::JointPositionController

File: gz-sim-joint-position-controller-system

Controlled joints:
arm_base_forearm_joint
forearm_hand_joint

Control type: Position control (P controller)

Gains:
arm_base_forearm_joint → p_gain = 5.0
forearm_hand_joint → p_gain = 3.0

🛞 Differential Drive (Mobile Base)
Plugin: gz::sim::systems::DiffDrive
File: gz-sim-diff-drive-system

Controlled joints:
base_left_wheel_joint
base_right_wheel_joint

Parameters:
Wheel separation: 0.45 m
Wheel radius: 0.1 m

Frames:
odom
base_footprint

📡 Joint State Publishing
Plugin: gz::sim::systems::JointStatePublisher

File: gz-sim-joint-state-publisher-system

Publishes joint states for:
Arm joints
Wheel joints
Joint states are consumed by robot_state_publisher and visualized in RViz2.

⚠️ Notes
Always source the workspace before running launch files.
Ensure Gazebo Harmonic plugins are compatible with ROS 2 Jazzy.
Camera frame IDs and topic names can be changed in the URDF/XACRO.
RViz2 display configuration can be customized further.

📄 License
This project is licensed under the MIT License.
You are free to use, modify, and distribute this software with proper attribution.
See the LICENSE file for details.
