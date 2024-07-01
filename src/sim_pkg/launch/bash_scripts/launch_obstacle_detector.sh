#!/bin/bash
roslaunch obstacle_detector nodes.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)