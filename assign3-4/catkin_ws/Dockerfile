# The 'robot' image already includes many important packages
FROM ros:melodic-robot-bionic

# Run this install pip
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install python-pip -y

# Install Python packages
COPY ./requirements.txt /
RUN pip install -r requirements.txt

# Change working directory
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws

# Add files with ROS dependencies. This .tar is generated using `sh_create_tar_ros_cmake.sh`
ADD ./ig_src_dependencies.tar.gz .

# Install all dependencies from the packages inside the 'src' folder
RUN rosdep install --from-paths src --ignore-src -r -y

# Run first catkin_make
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make; source devel/setup.bash"

# Add source code
COPY . .

# Set default ENV to "development"
ARG ENV_ARG="deploy"
# ENV_ARG values:
# "development": when we want to be able to build and debug inside the container
# "deploy": when we JUST want to be able to run inside the container

ENV ENV=$ENV_ARG
ENV ROS_WS=/home/catkin_ws

# Run second catkin_make (should be faster)
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make; source devel/setup.bash"
