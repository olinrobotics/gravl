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
    && apt-get install -y wget \
    && wget -O libsbp.tar.gz https://github.com/swift-nav/libsbp/archive/v$LIBSBP_V.tar.gz \
    && tar xvf libsbp.tar.gz \
    && cd libsbp-$LIBSBP_V/c \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && cd /home/$DOCKER_USER/catkin_ws \
    && wstool update -t src \
    && rosdep update \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep install -iry --from-paths src \
    && catkin_make -j1 \
    && source /home/$DOCKER_USER/catkin_ws/devel/setup.bash \
    && echo "source ~/catkin_ws/devel/setup.bash" >> /home/$DOCKER_USER/.bashrc \
    && chown -R $DOCKER_USER /home/$DOCKER_USER'

WORKDIR /home/$DOCKER_USER/catkin_ws
USER $DOCKER_USER

WORKDIR /home/$DOCKER_USER
