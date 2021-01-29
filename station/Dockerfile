FROM ros:noetic-ros-base

COPY . .

EXPOSE 4000

RUN chmod +x /ros_entrypoint.sh

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"

ENTRYPOINT ["/ros_entrypoint.sh"]
