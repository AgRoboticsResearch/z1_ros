# Start a servo control implemented by Unitree-z1


## 1. start z1-gazebo
```bash
roslaunch unitree_gazebo z1.launch
```

## 2. start z1-ctrl-fsm
```bash
cd z1_controller/build
./sim_ctrl # for gazebo
./z1_ctrl  # for real robot
```

## 3. build z1-python-interface

```bash
cd z1_sdk # use the repo forked in AgRobotics
mkdir build
cd build
cmake ..
make
cp z1_arm_interface.cpython-38-x86_64-linux-gnu.so /path/to/package/z1_sdk/lib/
```


## 4. start switch controller
```bash
roslaunch z1_hw z1_servo.launch 
```

## 5. Complete Control Stack with MoveIt and Trajectory Control

### 5.1 Launch the full simulation with MoveIt integration
```bash
roslaunch z1_bringup sim_arm_control_test.launch 
```

### 5.2 Topic-based joint control
You can control the robot by sending commands to the `/joint_commands` topic using the `z1_topic_control.py` node:

```bash
# Start the topic control node (converts joint commands to smooth trajectories)
rosrun z1_hw z1_topic_control.py

# Use the joint_state_publisher_gui to control the arm interactively
rosrun joint_state_publisher_gui joint_state_publisher_gui joint_states:=/joint_commands
```

The `z1_topic_control.py` node handles:
- Converting joint position commands to smooth trajectories
- Automatically switching to the appropriate controller
- Calculating appropriate movement times based on distance
