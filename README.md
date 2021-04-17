# diffbot2_robot
ROS2 packages of a differential robot


### diffbot2_description

This package has the xacro description of the diffbot2 robot and some usefull `launch` files. To just spawn the robot into the `robot_description` parameter, run:

```bash
  $ ros2 launch diffbot2_description spawn_robot.launch.py
```

The `view_robot.launch.py` start RViz with some basic configurations. By default, this file doesn't spawn the robot but there is an argument *`spawn_robot`* that enables spawn the robot and view it with RViz. Just start RViz:

```bash
  $ ros2 launch diffbot2_description view_robot.launch.py
```
Start RViz and spawn diffbot2 robot:

```bash
  $ ros2 launch diffbot2_description view_robot.launch.py spawn_robot:=true
``` 
