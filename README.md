# rover_21.5_state_machine_sim
State machine configured for simulation environment..

## How to run everything and get rover going?

In order to run simulation without steering:
```
roslaunch rover_21_description_velodyne gazebo.launch
```

For odometry:
```
rosrun rover_21_control rover_sim_odom.py
```

Localization should be run after odometry, please wait for odometry o work before run this, it sometimes cause problems.
```
roslaunch rover_21_localization localization.launch
```

Run to use rover autonomously:
```
roslaunch locomove_base locomove.launch
```

State machine:
```
rosrun rover_22_state_machine main.py
```
If state machine code does not stop, please shut down the sub-terminal window using ctrl+w, we are working on the issue.
