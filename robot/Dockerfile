FROM ros:noetic-ros-base

COPY . .

EXPOSE 5000

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
