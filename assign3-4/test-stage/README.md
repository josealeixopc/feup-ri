# Stage

## Resources

- https://docs.google.com/document/d/1Z9a7_pUjAzffjLQwJS3TbPZKW0F-7FDCU_35fqv-Ycs/edit#heading=h.5cn7d0u38h0k
- REALLY HANDY: https://player-stage-manual.readthedocs.io/en/stable/
- Stage manual: http://rtv.github.io/Stage/group__model.html

## Installing Stage

Stage is a third-party simulator that **does not use Catkin** to be built and installed. Therefore you'll have to follow the instructions in the `INSTALLING_STAGE.md` file.

## External Packages

The `external-packages` folder in my Catkin workspace contains packages that were not built by me. Because of problems or for ease of use, I have not installed those packages globally. Therefore their dependencies must be installed and the packages built.

To install the dependencies of all packages in the workspace, run this on the `catkin_ws` directory:

```
rosdep install --from-paths src --ignore-src -r -y
```

Then don't forget to build and run the `setup.bash` script:

```
catkin_make
source devel/setup.bash
```

### Installing Stage ROS

`stage_ros` is available [here](http://wiki.ros.org/stage_ros). It has some nice resources also related to the Stage simulator.

## Creating a simulation in Stage

### Creating my own world

Check the comments on the `simple.world` file inside my internal package.