FROM ros:foxy

ARG USERNAME=mcav
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WS_DIR=/home/${USERNAME}/ros2_ws
ENV SRC_DIR=${WS_DIR}/src

RUN apt-get update && apt-get install -y \
	software-properties-common \
  vim \
  tmux \
	wget \
	kmod \
  iproute2 \
	&& wget https://bootstrap.pypa.io/get-pip.py \
	&& python3 get-pip.py \
	&& rm get-pip.py \
	&& rm -rf /var/lib/apt/lists/* \
  && apt-get clean

RUN groupadd --gid $USER_GID $USERNAME \
	&& useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
	&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Prepare ROS2 overlay workspace
RUN mkdir -p ${SRC_DIR}

# Clone external source
WORKDIR ${SRC_DIR}
COPY streetdrone-hil.repos .
RUN mkdir external/ && vcs import external/ < streetdrone-hil.repos --recursive
RUN rm streetdrone-hil.repos

WORKDIR ${WS_DIR}
RUN apt-get update && rosdep update && DEBIAN_FRONTEND=noninteractive rosdep install --include-eol-distros --from-paths src --ignore-src -r --default-yes \
	&& apt-get install -y \
	ros-${ROS_DISTRO}-can-msgs \
	&& rm -rf /var/lib/apt/lists/* \
	&& python3 -m pip install carla==0.9.13 \
	&& apt-get clean

# Project-specific depenedency installs
WORKDIR ${SRC_DIR}/external/carla-ros-bridge
ENV ROS_PYTHON_VERSION="3"
ENV ROS_VERSION="2"
RUN ./install_dependencies.sh

# Build project
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ${WS_DIR}; colcon build --symlink-install --packages-up-to carla_ros_bridge sd_msgs sd_vehicle_interface hardware_in_loop'

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source ${WS_DIR}/install/setup.bash" >> /home/${USERNAME}/.bashrc
# Change prompt to show we are in a docker container
RUN echo "export PS1='\[\e]0;\u@docker: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@docker\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/${USERNAME}/.bashrc
# Change owner of the files to non-root user
RUN chown -R ${USERNAME} /home/${USERNAME}

WORKDIR ${SRC_DIR}
USER $USERNAME
CMD ["/bin/bash"]
