FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y iproute2 python3-pip && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# pymavlink backs the MAVLink comparison panel. Pinned to the version the GroundStation client
# is tested against, so the webapp decodes exactly what the ground station does.
RUN pip3 install --no-cache-dir "pymavlink>=2.4.49" "requests>=2.31"

# The shared GroundStation client, so the panel reads both wires through the same code the rest
# of the ground station uses rather than a second implementation that could drift from it.
COPY Python/wildbridge_groundstation ./wildbridge_groundstation

COPY video_test/webapp/server.py ./server.py
COPY video_test/webapp/mavlink_debug.py ./mavlink_debug.py
COPY video_test/webapp/video_events.py ./video_events.py
COPY video_test/webapp/public ./public

ENV PORT=8090

CMD ["python3", "server.py"]
