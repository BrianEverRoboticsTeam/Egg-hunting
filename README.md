# Egg-hunting
Course work for CMPUT 412 - Experimental Mobile Robotics (Winter 2017), Competition 4 @ University of Alberta, Canada 

Full documentations could be found at [here(competition 4)](https://brainever.wordpress.com/2017/04/11/egg-hunting-attempt-1/) and [here(competition 5)](https://brainever.wordpress.com/2017/04/11/egg-hunting-attempt-2/).

# Setup

Copy the following commands and paste them into terminal and they should magically work.
```
git clone https://github.com/NunchakusLei/Egg-hunting.git turtlebot_egg_hunting_project
cd turtlebot_egg_hunting_project
catkin_make
source devel/setup.bash
```

The following are detail explanation of the commands above.

- Clone the repo by
```
git clone https://github.com/NunchakusLei/Egg-hunting.git turtlebot_egg_hunting_project
```

- Goes into the folder just clone by
```
cd turtlebot_egg_hunting_project
```

- Catkin Make workspace by
```
catkin_make
```

- Source the workspace by
```
source devel/setup.bash
```

Now you are good to go.

# Launch

terminal 1
```
roslaunch state_machine_controller control.launch
```

terminal 2
```
roslaunch state_machine_controller side_view.launch 
```

terminal 3
```
roslaunch state_machine_controller navi.launch 
```

terminal 4
```
roslaunch state_machine_controller front_view.launch 
```

terminal 5
```
rosrun state_machine_controller state_machine.py
```

**Please note that**, you must launch front_view.launch after navi.launch have been successfully launch. Otherwise, the 3d_sensor might not work properly. 


