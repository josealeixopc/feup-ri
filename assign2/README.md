# assign2

The assignement 2 for robotics consisted of creating a **reactive** robot that could follow a wall using **ROS**. The bot should have differential driving.

For this assignment, we built a Gazebo model which interacts with a ROS program to follow a wall closely.

**DO NOT FORGET!!** Run `source catkin_ws/devel/setup.bash` in every terminal that is using the `reactive_bot` packages.

## Structure

- `src`
    - `reactive_bot_description` -- contains the ROS robot related files
        - `launch` -- launch files for controllers, spawn and visualizaing robot
        - `rviz` -- configuration files for Rviz
        - `scripts` -- robot Python controllers
        - `urdf` -- robot specification
    - `reactive_bot_gazebo` -- contains the Gazebo environment related files
        - `launch` -- Gazebo launch specification files 
        - `worlds` -- simulation world specification files

## Requirements

- Python 2.7
- ROS Melodic
- `gazebo_ros` packages

## Building

Run the following command inside the `catkin_ws` respository:

```catkin_make```

## Running

All these commands should be ran inside the `catkin_ws` folder.

### Starting Gazebo

To start Gazebo with all necessary ROS connections provided by the `gazebo_ros` package, execute:

```roslaunch gazebo_ros <world-launch-file>```

For a simple world, do:

```roslaunch gazebo_ros empty_world.launch```

For our custom world, do:

```roslaunch reactive_bot_gazebo reactive_bot.launch```

### Spawning a reactive bot

To spawn any model onto the **already running Gazebo world**:

```rosrun gazebo_ros spawn_model -file <path-of-model> -<type-of-model-extension> -x <x-coord> -y <y-coord> -z <z-coord> -model <name-of-entity>```

The `name-of-entity` parameter **can not be the same as an entity that already exists** in the world.

For help:

```rosrun gazebo_ros spawn_model -h```

To spawn our custom reactive robot into the **already running world**:

```roslaunch reactive_bot_description spawn.launch model:=<name-of-entity>```

**WARNING**: Because of a problem with the differential drive plugin, when the robot model is deleted, `ros_gazebo` crashes.

### Controlling the bot

Using keys:

```rosrun reactive_bot_description diff_wheeled_robot_key.py```

Using the controller:

```rosrun reactive_bot_description controller.py```

### Visualizing the bot

To visualize the bot, you can use Rviz to recreate the URDF information.

A bug with URDF makes it impossible to visualize the bot, unless you export the following environmental variable:

```export LC_NUMERIC="en_US.UTF-8"```

Then you may use the Rviz laucher:

```roslaunch reactive_bot_description rviz.launch```

You must change the `Fixed Frame` from `Global Options` to `link_base` and then add a display of a `Robot Model` (if a config file is given to the Launch file, this can be done automatically). 
