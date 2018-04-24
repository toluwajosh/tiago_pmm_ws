### TIAGo PMM Workspace
*ROS Workspace for the IROS 2018 Hackathon*

Assuming you already have a working TIAGo installation:
*After cloning this repo, do the following*
* Run

`rosinstall src /opt/ros/indigo tiago_public.rosinstall`

```python
# Use rosdep to install all dependencies. Just in case.
```

`rosdep install --from-paths src --ignore-src -y --rosdistro indigo --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit sensor_to_cloud"`

`source /opt/ros/indigo/setup.bash`

```python
# Initialize workspace and build packages
```

`catkin init`

`catkin build`

`source ~/tiago_pmm_ws/devel/setup.bash`

For the instructions to install TIAGo with ROS from scratch, see the following pages:
* [Official Installation Page](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/TiagoSimulation). *May run into some problem.*

* [Instructions from a thread](https://answers.ros.org/question/283875/dependency-error-while-installing-tiago/). *Useful in case of dependency issues.*

Please share any other issues encountered.

#### To contribute;
Please create your own branch and work on it, only send merge requests to the devel branch. 
All accepted updates will be merged to the master branch.


#### Tasks Division:
|Main Task |Subdivision |Remark |Person |
|:-----|:-----|:-----|:-----|
|**Perception** |	|	|	|
|				|Object Localization	|	|Su, Peng	|
|				|Object Pose Estimation	|	|Su, Peng	|
|				|Liquid Level Estimation	|	|Bian	|
|**Trajectory**|	|	|
|				|Planning	|	|Joshua	|
|				|Generation	|	|Joshua	|
|				|Gripper Control	|	|Kingkan	|
|**Simulation and System Integration** |	|	|
|				|Liquid	|	|Joe	|
|				|Object Interaction	|	|	|
|				|System Integration	|	|Chiba	|


#### Essential Tutorials
* [Planning in cartesian space](http://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Planning_cartesian_space)
* [Pick and Place demo](http://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Pick_place)
* [Play predefined upper body motions](http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion)
* [All the tutorials](http://wiki.ros.org/Robots/TIAGo/Tutorials)
* [Importing external 3D model into gazebo simulation](https://www.youtube.com/watch?v=aP4sDyrRzpU)