ARG DISTRO=kinetic

FROM ros:$DISTRO

ARG USER=kubo
ARG LIBSBP_V=2.3.10

MAINTAINER Kawin Nikomborirak concavemail@gmail.com

# Prerequisites
# The wget to the make install is to install libsbp for swiftnav.
RUN bash -c \
    'apt-get update \
    && apt-get install -y wget \
    && useradd -lmG video $USER \
    && wget -O libsbp.tar.gz https://github.com/swift-nav/libsbp/archive/v$LIBSBP_V.tar.gz \
    && tar xvf libsbp.tar.gz \
    && cd libsbp-$LIBSBP_V/c \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && mkdir -p /home/$USER/catkin_ws/src/gravl'

WORKDIR /home/$USER/catkin_ws
COPY . src/gravl/
COPY gravl.rosinstall src/.rosinstall

# Install dependencies
RUN bash -c \
    'wstool update -t src \
    && chown -R $USER . \
    && rosdep update \
    && rosdep install -iry --from-paths src'

USER $USER

# Catkin make
RUN bash -c \
    'source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin_make \
    && source devel/setup.bash \
    && echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc'

WORKDIR /home/$USER
