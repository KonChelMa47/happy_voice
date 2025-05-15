#!/bin/bash
cd ~/main_ws/src/happy_voice/happy_stt
docker compose build
docker-compose run --rm happy_stt bash -c "\
  source /opt/ros/humble/setup.bash && \
  source /opt/hm_msgs_ws/install/setup.bash && \
  ros2 run happy_stt stt"

