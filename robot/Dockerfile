FROM ros:noetic-ros-core

RUN apt update

RUN apt install -y build-essential

RUN mkdir -p /root/catkin_ws/src

COPY . /root/catkin_ws/src

WORKDIR "/root/catkin_ws"

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace /root/catkin_ws/src && catkin_make"

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
