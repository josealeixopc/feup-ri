# assign2

The assignement 2 for robotics consisted of creating a **reactive** robot that could follow a wall using **ROS**. The bot should have differential driving.

For this assignment, we built a Gazebo model which interacts with a ROS program to follow a wall closely.

**DO NOT FORGET!!** Run `source catkin_ws/devel/setup.bash` in every terminal that is using the `reactive_bot` packages.

## Starting Gazebo

To start Gazebo with all necessary ROS connections provided by the `gazebo_ros` package, execute:

```roslaunch gazebo_ros <world-launch-file>```

For a simple world, do:

```roslaunch gazebo_ros empty_world.launch```

For our custom world, do:

```roslaunch reactive_bot_gazebo reactive_bot.launch```

## Spawning a reactive bot

To spawn any model onto the **already running Gazebo world**:

```rosrun gazebo_ros spawn_model -file <path-of-model> -<type-of-model-extension> -x <x-coord> -y <y-coord> -z <z-coord> -model <name-of-entity>```

The `name-of-entity` parameter **can not be the same as an entity that already exists** in the world.

For help:

```rosrun gazebo_ros spawn_model -h```

To spawn our custom reactive robot into the **already running world**:

```rosrun gazebo_ros spawn_model -file `rospack find reactive_bot_gazebo`/models/my_reactive_robot.sdf -sdf -x 0 -y 0 -z 1 -model my_reactive_robot_0```