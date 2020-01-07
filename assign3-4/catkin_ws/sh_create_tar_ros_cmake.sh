#!/bin/bash

tar \
--exclude=src/external-packages/openai_ros/openai_ros/src/openai_ros/task_envs/turtlebot3_my_envs \
--exclude=src/internal-packages \
--exclude="*.pgm" \
--exclude="*.stl" \
--exclude="*.dae" \
-cvjf ig_src_dependencies.tar.gz src