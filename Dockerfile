FROM osrf/ros:noetic-desktop-full

WORKDIR /workspace
COPY . .

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge

RUN pip3 install transitions 