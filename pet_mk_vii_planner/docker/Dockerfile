FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# pico SDK setup based on:
# https://robofoundry.medium.com/raspberry-pi-pico-ros2-via-micro-ros-actually-working-in-1-hr-9f7a3782d3e3
RUN apt-get update && sudo apt install -y build-essential cmake gcc-arm-none-eabi libnewlib-arm-none-eabi

RUN groupadd --system pi && useradd --create-home --shell /bin/bash --gid pi pi

RUN echo "source /ros2_ws/install/setup.bash" >> /home/pi/.bashrc

USER pi