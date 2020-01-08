# The 'robot' image already includes many important packages
FROM osrf/ros:melodic-desktop-full-bionic

# Set default ENV to "deploy"
ARG ENV_ARG="deploy"
# ENV_ARG values:
# "development": when we want to be able to build and debug inside the container
# "deploy": when we JUST want to be able to run inside the container

ARG ROS_WS_ARG="/home/catkin_ws"

ENV ENV=$ENV_ARG
ENV ROS_WS=$ROS_WS_ARG

# Run this install pip
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install python-pip -y

# Install Python packages
COPY ./requirements.txt /
RUN pip install -r requirements.txt

# Change working directory
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Add files with ROS dependencies. This .tar is generated using `sh_create_tar_ros_cmake.sh`
ADD ./ig_src_dependencies.tar.gz .

# Install all dependencies from the packages inside the 'src' folder
RUN rosdep install --from-paths src --ignore-src -r -y

# Run first catkin_make
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make; source devel/setup.bash"

# Add source code
COPY . .

# Run second catkin_make (should be faster)
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make; source devel/setup.bash"

ENTRYPOINT ["/bin/bash"]
