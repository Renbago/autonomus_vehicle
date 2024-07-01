#!/bin/bash

rviz -d $(find sim_pkg)/launch/rviz_config.rviz 2> >(grep -v TF_REPEATED_DATA buffer_core)
