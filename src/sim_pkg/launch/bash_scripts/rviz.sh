#!/bin/bash

PKG_PATH=$(rospack find models_pkg)
if [ -z "$PKG_PATH" ]; then
    echo "Error: models_pkg not found in ROS workspace."
    exit 1
fi

TEXTURE_PATH="$PKG_PATH/track/materials/textures/new.png"

if [ ! -f "$TEXTURE_PATH" ]; then
    echo "Error: $TEXTURE_PATH does not exist."
    exit 1
fi

echo "Found PNG file: $TEXTURE_PATH"

# Dynamically generate a YAML file instead of using the creating one for just map
MAP_YAML_PATH="/tmp/map.yaml"
cat <<EOF > "$MAP_YAML_PATH"
image: $TEXTURE_PATH
resolution: 0.002125  
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.5
EOF

echo "Generated YAML file at: $MAP_YAML_PATH"
cat "$MAP_YAML_PATH"

echo "Starting map_server with map: $MAP_YAML_PATH"
rosrun map_server map_server "$MAP_YAML_PATH" &

sleep 2
if ! rostopic list | grep -q "/map"; then
    echo "Error: /map topic not found. Map server might have failed."
    exit 1
fi

echo "/map topic successfully published."

CONFIG_PATH="$(rospack find sim_pkg)/launch/bash_scripts/rviz_config.rviz"
if [ ! -f "$CONFIG_PATH" ]; then
    echo "Error: RViz config file not found at $CONFIG_PATH."
    exit 1
fi

echo "Resolved RViz config path: $CONFIG_PATH"

rviz -d "$CONFIG_PATH" 2> >(grep -v TF_REPEATED_DATA buffer_core)
