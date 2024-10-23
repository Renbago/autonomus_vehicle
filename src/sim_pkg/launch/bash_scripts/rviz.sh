#!/bin/bash

CONFIG_PATH="$(rospack find sim_pkg)/launch/bash_scripts/rviz_config.rviz"
echo "Resolved RViz config path: $CONFIG_PATH"

rviz -d $CONFIG_PATH 2> >(grep -v TF_REPEATED_DATA buffer_core)
