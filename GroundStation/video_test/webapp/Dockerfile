FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y iproute2 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY server.py ./server.py
COPY video_events.py ./video_events.py
COPY public ./public

ENV PORT=8090

CMD ["python3", "server.py"]
