## A Demonstration of our robot Talos doing BMT test


#### 0. Preview

First of all, clone it to your ros workspace source file then go to your root workspace and type catkin_make and after that source it.
Now you can follow the steps to view the demo BMT test!

#### 1. Launch our world and Talos (add robocup objects)

In order to start the simulation go to your terminal and hit ``` roslaunch demo_navi mybot_world.launch```. After to the gazebo insert then Add Path and click object_sdf directory. (demo_navi/src/gazebo_worlds/)

#### 2. Merge our two LaserScans

Because we have two LiDAR we must somehow merge their scan's so launch
```roslaunch ira_laser_tools laserscan_multi_merger.launch```.

#### 3. Launch amcl and move base

Finally we can see our robot moving autonomously just launch amcl ```roslaunch amcl move_base_teb.launch```.

#### 4. Launch the arm

```roslaunch arm widowx_arm_planning_execution.launch```.

#### 5. Launch flexbe

```roslaunch flexbe_app flexbe_full.launch```. Click Load Behaviour then Final_BMT_Test at the bottom right, finally go to Runtime Controll and hit Start
Execution. (Note we will add this behaviour as soon as possible!)

#### 6. Run the bag file

Go to the bags_DIR directory and hit```rosbag play wm21test_DIR_BMT.bag ```. And the robot should start the BMT test!

### 7. Conclusion

The simulation is in an early stage a lot of things need to be optimized from navigation, manipulation, state-machine and of course vision. We had troubles with 
the transformations between the arm and the camera so that's why we didn't managed to complete the BMT task. Overall the results until now are satsfying (most of nav goals were successful)for us and we will continue to add new features and optimize it! 


:warning: *Every bug you encouter, address it to the Issues section above.* :warning:

Thank you!


### Sources from GitHub:

- https://github.com/GuiRitter/OpenBase
- https://github.com/iralabdisco/ira_laser_tools
- https://github.com/ros-perception/slam_gmapping
- https://github.com/ros-planning/navigation
- https://github.com/pal-robotics/realsense_gazebo_plugin
- https://github.com/issaiass/realsense2_description
- https://github.com/FlexBE/flexbe_app
- https://github.com/robocup-at-work/atwork-commander
