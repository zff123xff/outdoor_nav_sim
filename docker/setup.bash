# Source only container-owned Classic/ROS paths, then the container-built overlay.
# Do not inherit a host Gazebo Sim/Ignition installation or old library paths.
unset GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH GAZEBO_INSTALL_PREFIX
unset GAZEBO_MODEL_DATABASE_URI GAZEBO_MASTER_URI
source /usr/share/gazebo/setup.sh || return 1
# Host networking also shares TCP ports. Keep this stack off Classic's host
# default 11345; all GUI clients/terminals use the same explicit container URI.
export GAZEBO_MASTER_URI="${DOCKER_GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
source /opt/ros/humble/setup.bash || return 1
if [[ -r /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash || return 1
fi
# ros_env preserves explicit DDS settings and otherwise selects this repo's UDP profile.
if [[ -r /workspace/ros_env.sh ]]; then
    source /workspace/ros_env.sh || return 1
fi
