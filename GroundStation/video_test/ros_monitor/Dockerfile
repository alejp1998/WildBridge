# WildBridge ROS bridge monitor container.
# Build context is the repository root (./) so it can COPY the shared
# GroundStation packages; the root .dockerignore keeps the context small.
FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY GroundStation/video_test/ros_monitor/requirements.txt /opt/wildbridge/requirements.txt
RUN pip3 install --no-cache-dir -r /opt/wildbridge/requirements.txt

# Shared GroundStation client used by the ROS controller and the monitor.
# Installed as a distribution, so both import it from site-packages.
COPY GroundStation/Python /opt/wildbridge/groundstation
RUN pip3 install --no-cache-dir /opt/wildbridge/groundstation

# ROS dji_controller node (publishes telemetry + state/settings), built as a
# real colcon package so `ros2 launch`/`ros2 run` can resolve its `dji_node`
# executable the same way the rest of the ROS 2 stack does (see
# GroundStation/ROS/wildview_bringup's launch files).
COPY GroundStation/ROS/dji_controller /opt/ros_ws/src/dji_controller
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/ros_ws \
    && colcon build --packages-select dji_controller --symlink-install

# The auto-discovery launch file that starts one namespaced dji_node per
# discovered drone, unmodified apart from the added periodic re-scan.
COPY GroundStation/ROS/wildview_bringup/launch/auto_discovery_native.launch.py /opt/wildbridge/launch/auto_discovery_native.launch.py

COPY GroundStation/video_test/ros_monitor/ros_monitor.py /opt/wildbridge/ros_monitor/ros_monitor.py
COPY GroundStation/video_test/ros_monitor/entrypoint.sh /opt/wildbridge/entrypoint.sh

RUN chmod +x /opt/wildbridge/entrypoint.sh

ENTRYPOINT ["/opt/wildbridge/entrypoint.sh"]
