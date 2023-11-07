FROM osrf/ros:humble-desktop-full

# Install Nano
RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/bin/bash", "entrypoint.sh" ]