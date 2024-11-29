#!/bin/bash

PACKAGE_PATH=$(rospack find motion_planning_python)

cp $PACKAGE_PATH/config/rviz/custom_config.rviz /tmp/default.rviz

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/default.rviz