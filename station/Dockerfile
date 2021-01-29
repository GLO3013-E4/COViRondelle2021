FROM ros:noetic-ros-base

COPY . .

EXPOSE 4000

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
