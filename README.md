# ddMultiagentCoordinationSym
Multiagent Control System Simulator for Differential Drives robots

Implementation in ROS2 of the results and symulations of the IEEE article [J. Hu, P. Bhowmick and A. Lanzon, "Group Coordinated Control of Networked Mobile Robots With Applications to Object Transportation," in IEEE Transactions on Vehicular Technology, vol. 70, no. 8, pp. 8269-8274, Aug. 2021, doi: 10.1109/TVT.2021.3093157.](ieeexplore.ieee.org/abstract/document/9468402)

Later included results from the IEEE article [J. Hu, P. Bhowmick and A. Lanzon, "Distributed Adaptive Time-Varying Group Formation Tracking for Multiagent Systems With Multiple Leaders on Directed Graphs," in IEEE Transactions on Control of Network Systems, vol. 7, no. 1, pp. 140-150, March 2020, doi: 10.1109/TCNS.2019.2913619.](ieeexplore.ieee.org/document/8700227)

# First Steps
## Robot Spawning
First of all i have designed a differential drive robot similar to the one in the article, then i've written a launch file that spawns 6 identical robots with progressive namespaces.
![6 Robots in Gazebo](./img/gazebo_6_robots_spawn.jpg)
You can launch the launchfile with the command:
`ros2 launch multi_robots launch_sim.launch.py`


Then if i list all the nodes i get:

> /gazebo  
> /robot1/diff_drive  
> /robot1/robot_state_publisher  
> /robot2/diff_drive  
> /robot2/robot_state_publisher  
> /robot3/diff_drive  
> /robot3/robot_state_publisher  
> /robot4/diff_drive  
> /robot4/robot_state_publisher  
> /robot5/diff_drive  
> /robot5/robot_state_publisher  
> /robot6/diff_drive  
> /robot6/robot_state_publisher  

And if i list the topics:
> /clock  
> /parameter_events  
> /performance_metrics  
> /robot1/cmd_vel  
> /robot1/joint_states  
> /robot1/odom  
> /robot1/robot_description  
> /robot1/tf  
> /robot1/tf_static  
> /robot2/cmd_vel  
> /robot2/joint_states  
> /robot2/odom  
> /robot2/robot_description  
> /robot2/tf  
> /robot2/tf_static  
> /robot3/cmd_vel  
> /robot3/joint_states  
> /robot3/odom  
> /robot3/robot_description  
> /robot3/tf  
> /robot3/tf_static  
> /robot4/cmd_vel  
> /robot4/joint_states  
> /robot4/odom  
> /robot4/robot_description  
> /robot4/tf  
> /robot4/tf_static  
> /robot5/cmd_vel  
> /robot5/joint_states  
> /robot5/odom  
> /robot5/robot_description  
> /robot5/tf  
> /robot5/tf_static  
> /robot6/cmd_vel  
> /robot6/joint_states  
> /robot6/odom  
> /robot6/robot_description  
> /robot6/tf  
> /robot6/tf_static  
> /rosout  
> /tf

If you want to play around driving the robots you can do it using `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot1/cmd_vel` and substituting to `robot1` the namespace of the robot that you want to move.

If you want to spawn a different number of robots, or spawn them at different distance you just have to edit the [launch_config.yaml](./config/config.yaml) file.

## Functions
The second step has been to write the functions needed to calculate matrices, arrays and all the other values that you can need in the program.

You can find thoose under [src/utilities.h](./src/utilities.h)

Here you can find the status of the function written associated to the formulas:

 - :white_check_mark: = Implemented and tested
 - :warning: = Implemented but not tested
 - :no_entry: = Not Implemented
 (testing has to be intended as a run with successfull result, not as a complete test)

| Number | Description | Name | Status |
| :--------: | :-------: | :-------: | :------: |
| 1 | Weights of Graph | getWeighMatrix | :warning: |
| - | Versor x of vehicle | xVehicleVersor | :warning: |
| - | Versor y of vehicle | yVehicleVersor | :warning: |
| - | Converts the velocity of the forward point to unicycle lagrangian coordinates | velocityToUnicycle | :warning: |
| - | Transform a polar vector in a cartesian one | polarToCartesian | :white_check_mark: |
| - | Get the position of the i'th robot in a circular formation | getRobotFormationPosition | :white_check_mark: |
| - | Get the rotation matrix around z | zRotation | :white_check_mark: |
| - | Get yaw (theta) angle from quaternion | quaternionToTheta | :white_check_mark: |
| - | Convert v and theta_dot to point vel | unicycleToVelocity | :white_check_mark: |

## Proportional Control
I've implemented a simple proportional gain contro, just to verify that everything will work, and obviously, it doesn't.

![plot](./img/plot_instable.png)
The plot shows that the angular velocity of the robot get very high. I need to investigate further the reasons of why it happens.

Just to see if i can stop the robot to rotate i implemented a different logic:

 - If i'm far from the point let the acceleration be constant and in direction of the point
 - If i'm closer than a threshold, let the acceleration be zero

That changes didn't let to anything, so i decided to go further.

I tried to give a constant value to cmd_acc equal to [0.0, 1.0] and see what happened.

`ros2 topic pub /robot1/cmd_acc std_msgs/msg/Float64MultiArray "{data: [0.0, 1.0]}"`

The effect was that the robot started to go in one direction, but it started to deviate in a random way. When i stopped giving that signal and gave [0.0, 0.0] instead, the robot continued moving.

So the next step were 2:
 - Add a static friction in the movements to eventually stop the robot
 - Control again the accel_to_cmd_vel.cpp code