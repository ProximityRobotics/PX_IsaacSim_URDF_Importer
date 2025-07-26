##############################################################################
##                                 Base Image                               ##
##############################################################################
FROM nvcr.io/nvidia/isaac-sim:4.5.0

ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
RUN apt update -y && apt install -y \
    locales \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Set UTF-8 supported locale
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN LANG=en_US.UTF-8

# Ensure that the Ubuntu Universe repository is enabled
RUN add-apt-repository universe

# Add the ROS 2 GPG key and add the repository to the sources list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

##############################################################################
##                               Install ROS 2                              ##
##############################################################################
RUN apt update -y && apt install -y --allow-downgrades \
    libbrotli1=1.0.9-2build6 \
    libfreetype6-dev \
    libfontconfig1-dev \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-trajectory-msgs \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-topic-based-ros2-control \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# Set Fast DDS profile
COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

# Export ROS_DOMAIN_ID
ARG DOMAIN_ID=0
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

# Source ROS 2
RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

##############################################################################
##                        Prepare Python Scripts                            ##
##############################################################################
ENV PYTHONPATH="/isaac-sim/project/scripts:"

RUN ./python.sh -m pip install imageio[ffmpeg]

ARG SRC_CONTAINER=/isaac-sim/project/scripts
RUN echo "alias run_simulation='/isaac-sim/python.sh ${SRC_CONTAINER}/run_simulation.py'" >> /etc/bash.bashrc
