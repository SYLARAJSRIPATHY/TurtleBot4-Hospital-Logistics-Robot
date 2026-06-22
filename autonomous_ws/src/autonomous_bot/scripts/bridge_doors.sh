#!/usr/bin/env bash
set -e
WORLD=default
DOORS=(
  door_main_north
  door_main_south
  door_wardA
  door_icu
  door_pharmacy
)

for d in "${DOORS[@]}"; do
  echo "Bridging $d ..."
  ros2 run ros_gz_bridge parameter_bridge \
    /world/$WORLD/model/$d/joint/panel_slide/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double &
done

wait
