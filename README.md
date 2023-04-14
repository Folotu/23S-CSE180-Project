# Intro to Robotics Final Project

### 23-Spring-CSE 180 Final Project
**Detecting Dynamic Obstacles in a Robot's Environment**

Start by navigating to into the MRTP folder, the build the gazebo environment with this command
~~~
 colcon build --packages-select gazeboenvs
~~~

Once the build is complete, source the package with this command
~~~
 . install/setup.bash
~~~
now set the following environmental variables 
~~~
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/23S-CSE180-Project/MRTP/src/gazeboenvs/models
~~~
the `GAZEBO_MODEL_PATH` varaible is assumming that you have cloned the repo into your home directory, adjust as needed.

Then launch the gazebo environment with this 
~~~
 ros2 launch gazeboenvs tb3_simulation.launch.py
~~~
Now open a new shell, and navigate into the FPWorkSpace directory, then run this command
~~~
 colcon build --packages-select finalproject
~~~
Once that is done building, then source the package, with this command 
~~~
. install/setup.bash
~~~
then run the navigator executable with this command 
~~~
ros2 run finalproject myTestNavigator
~~~
and it should start printing the cell inconsistencies it finds as it moves.
