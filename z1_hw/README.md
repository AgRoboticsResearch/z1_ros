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
