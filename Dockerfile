ARG ROS_DISTRO=kinetic

FROM ros:$ROS_DISTRO

ARG DOCKER_USER=kubo
ARG LIBSBP_V=2.3.10

MAINTAINER Kawin Nikomborirak concavemail@gmail.com

RUN bash -c \
    'useradd -lmG video $DOCKER_USER \
    && mkdir -p /home/$DOCKER_USER/catkin_ws/src/gravl'

COPY . /home/$DOCKER_USER/catkin_ws/src/gravl/
COPY gravl.rosinstall /home/$DOCKER_USER/catkin_ws/src/.rosinstall

RUN bash -c \
    'apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y wget \
    && apt-get install -y sudo \
    && cd /home/$DOCKER_USER/catkin_ws \
#    && wget -O libsbp.tar.gz https://github.com/swift-nav/libsbp/archive/v$LIBSBP_V.tar.gz \
#    && tar xvf libsbp.tar.gz \
#    && cd libsbp-$LIBSBP_V/c \
#    && mkdir build \
#    && cd build \
#    && cmake .. \
#    && make \
#    && make install \
    && cd /home/$DOCKER_USER/catkin_ws/src/gravl \
    && ./setup.sh \
    && cd /home/$DOCKER_USER/catkin_ws \
    && wstool update -t src \
    && rosdep update \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep install -iry --from-paths src \
# Need to setup package separately
    && cd /home/$DOCKER_USER/catkin_ws/src/ethz_piksi_ros/piksi_multi_rtk_ros \
    && source install/install_piksi_multi.sh \
    && cd /home/$DOCKER_USER/catkin_ws \
    && catkin_make -j1 \
    && source /home/$DOCKER_USER/catkin_ws/devel/setup.bash \
    && echo "source ~/catkin_ws/devel/setup.bash" >> /home/$DOCKER_USER/.bashrc \
    && chown -R $DOCKER_USER /home/$DOCKER_USER'

WORKDIR /home/$DOCKER_USER/catkin_ws
USER $DOCKER_USER

WORKDIR /home/$DOCKER_USER
