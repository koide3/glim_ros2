FROM koide3/gtsam_docker:focal_cuda11.2

RUN apt-get update \
  && apt-get upgrade -y \
  # install dependencies
  && apt-get install --no-install-recommends -y \
    curl libopencv-dev \
  # install ROS
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update \
  && apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    ros-galactic-ros-base ros-galactic-cv-bridge \
    ros-galactic-image-transport-plugins ros-galactic-tf2-ros \
  # clean
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/galactic/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/ros2_ws/install/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

COPY glim /root/ros2_ws/src/glim
COPY glim_ros2 /root/ros2_ws/src/glim_ros2

WORKDIR /root/ros2_ws
RUN /bin/bash -c '. /opt/ros/galactic/setup.bash; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=ON'

WORKDIR /root/ros2_ws/src/glim

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
