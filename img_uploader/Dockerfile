FROM rrdockerhub/ros-base-kinetic-amd64

LABEL authors = "Yu Okamoto <yu.okamoto@rapyuta-robotics.com>"

ENV ROS_HOME=/ros


RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    ros-kinetic-rospy \
    ros-kinetic-roslaunch \
    build-essential \
    ros-kinetic-actionlib-msgs \
    ros-kinetic-image-view \
    python-pip \
    python-wheel \
    python-snappy

RUN pip install minio

RUN mkdir /img

RUN git clone https://github.com/yuokamoto/io_use_case.git /ros/ws/src 

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
    && cd /ros/ws/src \
    && rm CMakeLists.txt \
    && catkin_init_workspace\
    && cd /ros/ws \
    && rosdep update \
    && rosdep install --from-paths /ros/ws/src -y --ignore-src \
    && catkin_make -DCATKIN_ENABLE_TESTING=False"

RUN cp /ros/ws/src/img_uploader/ros_entrypoint.sh .

ENTRYPOINT ["/ros_entrypoint.sh"]
