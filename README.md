## MRL-AMRL Traversal Map

A 2D Occupancy Grid for CostMap by specifying traversal and non-traversal area due to Avoid Obstacles for EXP 4&5 Avoid Holes & Avoid Terrains in rescue robot League for autonomous robots only.

### usage

On this assumption you have an Ubuntu 16.04(Xenial) and you have installed ROS Kinetic Kame. Also, you have an autonomous robot. You need a 2D Occupancy Grid to help your robot undrestand traversal area to navigate.

### Download the pakage
```markdown
cd ~/wokspace/src
git clone https://github.com/SaraTohidi/traversal_map.git
```
### Build the workspace
```markdown
cd ..
catkin_make
```
Now you may need to get some dependencies.

### Get some dependencies
```markdown
ros-kinetic-opencv3
ros-kinetic-pcl-ros
ros-kinetic-pcl-conversions 
ros-kinetic-pcl-msgs
ros-kinetic-octomap
```
