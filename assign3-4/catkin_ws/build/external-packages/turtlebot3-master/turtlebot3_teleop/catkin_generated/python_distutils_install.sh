#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/turtlebot3-master/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/lib/python2.7/dist-packages:/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/turtlebot3-master/turtlebot3_teleop/setup.py" \
    build --build-base "/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/build/external-packages/turtlebot3-master/turtlebot3_teleop" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install" --install-scripts="/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/install/bin"
