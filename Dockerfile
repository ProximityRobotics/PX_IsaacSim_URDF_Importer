FROM nvcr.io/nvidia/isaac-sim:4.2.0

ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -y && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN LANG=en_US.UTF-8

RUN apt update -y && apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update -y && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update -y && apt install -y ros-humble-desktop

RUN apt update -y && apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt ros-humble-rqt-common-plugins \
    ros-humble-navigation2 \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-control-msgs \
    ros-humble-moveit-msgs && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt update -y && apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-colcon-common-extensions \
    build-essential && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

ARG DOMAIN_ID=0
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

ENV PYTHONPATH="/isaac-sim/project/scripts:"

RUN ./python.sh -m pip install imageio[ffmpeg]

ARG SRC_CONTAINER=/isaac-sim/project/scripts
RUN echo "alias run_simulation='/isaac-sim/python.sh /isaac-sim/project/scripts/run_simulation.py'" >> /etc/bash.bashrc
