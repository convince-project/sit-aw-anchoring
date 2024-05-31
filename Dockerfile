# version: 0.1.0

FROM ros:humble-ros-base

ENV ROS_DISTRO=humble

ARG DEBIAN_FRONTEND=noninteractive

ARG OWLREADY2_VERSION=0.45

SHELL ["/bin/bash", "-c"]

RUN apt update &&\
	apt install -y --no-install-recommends git python3-pip openjdk-17-jre-headless

ENV XDG_RUNTIME_DIR=/tmp/runtime-root
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/utils/fastrtps-profile.xml

WORKDIR /root/ros2_ws

COPY ./src /root/ros2_ws/src

#Install python dependencies
RUN pip3 install --upgrade pip &&\
	pip3 install scipy &&\
	pip3 install owlready2==${OWLREADY2_VERSION}

#Install missing dependencies (most of them should be tackeld above to optimise build time)
RUN source /opt/ros/humble/setup.bash &&\
	sudo apt update &&\
	rosdep fix-permissions &&\
	rosdep update &&\
	rosdep install -i --from-path . --rosdistro humble -y

#Build the components (separate step to cache the rosdep installs)
RUN source /opt/ros/humble/setup.bash &&\
	colcon build --symlink-install

#Set the entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

#Set the .bashrc
COPY ./.bashrc /root/.bashrc
