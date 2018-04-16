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