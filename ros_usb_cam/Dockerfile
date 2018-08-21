FROM rrdockerhub/ros-base-kinetic-amd64

LABEL authors = "Yu Okamoto <yu.okamoto@rapyuta-robotics.com>"

ENV ROS_HOME=/ros

RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    ros-kinetic-rospy \
    ros-kinetic-roslaunch \
    build-essential \
    ros-kinetic-actionlib-msgs \
    ros-kinetic-usb-cam \
    python-pip \
    python-wheel \
    python-snappy

COPY ros_usb_cam/ros_entrypoint.sh .

ENTRYPOINT ["/ros_entrypoint.sh"]
