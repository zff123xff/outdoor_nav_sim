FROM docker.io/osrf/ros:humble-desktop-full-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

# Classic development headers/plugins, CHAMP controllers and the optional MQTT
# gateway's native libraries. Remaining manifest dependencies come from rosdep.
# Classic may return 255 for --version; validate its actual banner, not that code.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git sudo python3-colcon-common-extensions python3-rosdep \
    gazebo libgazebo-dev libboost-chrono-dev \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \
    ros-humble-rviz2 ros-humble-rqt ros-humble-rqt-common-plugins \
    ros-humble-rmw-fastrtps-cpp ros-humble-rmw-cyclonedds-cpp \
    libgl1-mesa-dri libgl1-mesa-glx mesa-utils xauth \
    libpaho-mqttpp-dev libpaho-mqtt-dev libjsoncpp-dev libyaml-cpp-dev \
    libssl-dev mosquitto python3-paho-mqtt python3-pymap3d python3-shapely \
    && test "$(. /etc/os-release && echo "$VERSION_CODENAME")" = jammy \
    && gazebo_version="$(gazebo --version 2>&1 || true)" \
    && printf '%s\n' "$gazebo_version" | grep -E '^Gazebo multi-robot simulator, version 11\.' \
    && rm -rf /var/lib/apt/lists/*

# Only dependency metadata enters the image; source/assets always use a bind mount.
COPY simulation/src/sim_bringup/package.xml /tmp/project-deps/simulation/src/sim_bringup/package.xml
COPY simulation/src/sim_robot/package.xml /tmp/project-deps/simulation/src/sim_robot/package.xml
COPY simulation/src/sim_world/package.xml /tmp/project-deps/simulation/src/sim_world/package.xml
COPY simulation/src/sim_lidar/package.xml /tmp/project-deps/simulation/src/sim_lidar/package.xml
COPY simulation/src/sim_localization/package.xml /tmp/project-deps/simulation/src/sim_localization/package.xml
COPY simulation/src/third_party/champ/champ/package.xml /tmp/project-deps/simulation/src/third_party/champ/champ/package.xml
COPY simulation/src/third_party/champ/champ_base/package.xml /tmp/project-deps/simulation/src/third_party/champ/champ_base/package.xml
COPY simulation/src/third_party/champ/champ_msgs/package.xml /tmp/project-deps/simulation/src/third_party/champ/champ_msgs/package.xml
COPY commands/src/outdoor_nav_mqtt_sim/package.xml /tmp/project-deps/commands/src/outdoor_nav_mqtt_sim/package.xml
COPY docker/rosdep-skip-keys.txt /usr/local/share/outdoor-nav/rosdep-skip-keys.txt

RUN test "$USER_UID" -gt 0 && test "$USER_GID" -gt 0 \
    && (getent group "$USER_GID" || groupadd --gid "$USER_GID" ros) \
    && useradd --uid "$USER_UID" --gid "$USER_GID" --create-home --shell /bin/bash ros \
    && echo 'ros ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros \
    && mkdir -p /workspace && chown "$USER_UID:$USER_GID" /workspace \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

USER ros
RUN rosdep update --rosdistro humble \
    && sudo apt-get update \
    && rosdep install --from-paths /tmp/project-deps/simulation/src /tmp/project-deps/commands/src \
       --ignore-src --rosdistro humble -y \
       --skip-keys "$(tr '\n' ' ' < /usr/local/share/outdoor-nav/rosdep-skip-keys.txt)" \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/project-deps

COPY docker/setup.bash /usr/local/share/outdoor-nav/setup.bash
COPY docker/entrypoint.sh /usr/local/bin/outdoor-nav-entrypoint
RUN echo 'source /usr/local/share/outdoor-nav/setup.bash' >> /home/ros/.bashrc
ENV OUTDOOR_NAV_SYSTEM_DEPS=1 QT_X11_NO_MITSHM=1
WORKDIR /workspace
ENTRYPOINT ["/bin/bash", "/usr/local/bin/outdoor-nav-entrypoint"]
CMD ["bash"]
