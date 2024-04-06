FROM nvidia-ros-noetic:latest
SHELL [ "/bin/bash", "-c" ]

ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=noetic

RUN useradd --create-home appuser

USER appuser
WORKDIR /appuser
RUN mkdir -p catkin_ws/src
COPY ./src catkin_ws/src

USER root
RUN apt-get update --fix-missing \
  && rosdep update
WORKDIR /appuser/catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN apt-get install ros-noetic-navigation -y

RUN apt-get install -y \
  google-mock \
  python3-sphinx \
  libboost-iostreams-dev \
  libeigen3-dev \
  libcairo2-dev \
  libceres-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  liblua5.2-dev \
  libprotobuf-dev

USER appuser
WORKDIR /home/appuser
RUN git clone https://github.com/osrf/gazebo_models.git
RUN mkdir -p .gazebo/models \
  && cp -r gazebo_models/* .gazebo/models \
  && cp -r /appuser/catkin_ws/src/me5413_world/models/* .gazebo/models
WORKDIR /appuser/catkin_ws
RUN source /opt/ros/noetic/setup.bash \
  && catkin config --install \
  && catkin build

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /appuser/catkin_ws/devel/setup.bash" >> ~/.bashrc
