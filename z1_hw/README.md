# Start a servo control implemented by Unitree-z1


## 1. start z1-gazebo
```bash
roslaunch unitree_gazebo z1.launch
```

## 2. start z1-controller state machine

```bash
cd ros_controller/build
./sim_ctrl # for gazebo controller
./z1_ctrl # for real robot controller
```


## 3. start switch controller
```bash
roslaunch z1_hw z1_servo.launch 
```
