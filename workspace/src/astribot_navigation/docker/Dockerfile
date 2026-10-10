ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
RUN apt-get update && apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions python3-pytest python3-numpy python3-yaml python3-scipy \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-tf2-sensor-msgs ros-humble-sensor-msgs-py ros-humble-pointcloud-to-laserscan \
  ros-humble-rosbag2-storage-mcap ros-humble-rosbag2-py ros-humble-diagnostic-msgs \
  && rm -rf /var/lib/apt/lists/*
# Pinned Humble upstream overlay with a narrowly scoped grid-bounds fix.
COPY docker/patch_slam_grid.py /tmp/patch_slam_grid.py
COPY docker/vendor/slam-toolbox-2.6.10.tar.gz /tmp/slam-toolbox.tar.gz
RUN echo "5cc31b72e89903ee1b2c44699e93a800e4e425f3b9eee17d7c0c20e12a66f8af  /tmp/slam-toolbox.tar.gz" | sha256sum -c - \
  && mkdir -p /slam_overlay/src/slam_toolbox \
  && tar -xzf /tmp/slam-toolbox.tar.gz -C /slam_overlay/src/slam_toolbox --strip-components=1 \
  && python3 /tmp/patch_slam_grid.py \
  && source /opt/ros/humble/setup.bash \
  && cd /slam_overlay && MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  && printf 'slam_toolbox 2.6.10 8293d21fe5c816d0405e0cc1f3eb46c70659c267 + TowerGO grid padding\n' > /slam_overlay/PROVENANCE.txt
WORKDIR /towergo
COPY . /towergo/src/astribot_navigation
RUN source /slam_overlay/install/setup.bash && colcon build --symlink-install --parallel-workers 2 --packages-ignore towergo_real towergo_livox_adapter \
  && dpkg-query -W > /towergo/dependency-versions.txt && cat /slam_overlay/PROVENANCE.txt >> /towergo/dependency-versions.txt
COPY docker/entrypoint.sh /towergo-entrypoint.sh
RUN chmod +x /towergo-entrypoint.sh
ENTRYPOINT ["/towergo-entrypoint.sh"]
CMD ["bash"]
