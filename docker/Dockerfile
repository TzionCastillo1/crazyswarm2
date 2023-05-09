FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND noninteractive 

RUN apt-get -qq update && \
	apt-get -qq upgrade && \
	apt-get install -y  apt-utils build-essential && \
	apt-get install -y python3-pip swig


RUN apt-get install -y  ros-humble-desktop python3-dev

#install Gazebo Garden
RUN apt-get update && \
	apt-get install  -y lsb-release wget gnupg && \
	wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
	apt-get update && \
	apt-get install -y gz-garden


ENV ROS_WS /opt/ros_ws
ARG ROSINSTALL=midas.rosinstall

COPY ${ROSINSTALL} ${ROSINSTALL}

RUN mkdir -p ${ROS_WS}/src && \
	vcs import ${ROS_WS}/src < ${ROSINSTALL}

WORKDIR $ROS_WS

RUN apt-get update && \
	. /opt/ros/humble/setup.sh && \
	export GZ_VERSION=garden && \
	rosdep install -r -y --ignore-src --from-paths src --rosdistro humble && \
	colcon build 

 
# Install crazyflie firmware for SITL
RUN cd && \
	apt-get install -y make gcc-arm-none-eabi && \
 	git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git && \
	cd crazyflie-firmware && \
	git submodule init && \
	git submodule update && \
	make cf2_defconfig && make -j 12 && \
	make bindings_python

RUN apt-get update && \
	pip3 install rowan && \
	apt-get install -y ros-humble-tf-transformations

RUN export PYTHONPATH=~/crazyflie-firmware/build:$PYTHONPATH

# Install RVIZ dependencies
RUN apt-get update && apt-get install -y \
		ros-humble-teleop-twist-keyboard \
		ros-humble-rviz2 \
		ros-humble-rviz-common \
		ros-humble-rviz-default-plugins \
		ros-humble-rviz-visual-tools \
		ros-humble-rviz-rendering \
		ros-humble-nav2-rviz-plugins && \
	apt-get upgrade -y && \
	apt-get autoremove -y && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS

RUN . $ROS_WS/install/setup.sh
