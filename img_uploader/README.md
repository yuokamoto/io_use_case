## img_uploader ##
### Summary  ###
Save image from web camera and upload to the minio server at regular intervals.

### Launch Files  ###

* img_uploader.launch: upload image to the minio server. Image is saved as `img_name` and uploaded to the minio server at `server_ep`. web-video-viewer runs at `<ip_address>:8080`
**Parameters**
   * `use_camera`: use web camera or not.  default is `false`.
  * `use_web_video_server`: use web video server.  default is `false`.
   * `img_upload_period`:   interval of the image upload[s]. default is `$(optenv IMG_UPLOAD_PERIOD 600)`
   * `img_name`: image file name to be upload default is `$(optenv IMAGE_SAVE_DIRECTORY /tmp/)img_to_be_upload.png`
   * `server_ep`:  default is `$(optenv ep http://locaohost:443)`

### Setup Workspaces ###
install minio client
```
sudo pip install minio
```

setup ROS workspace
```
source /opt/ros/kinetic/setup.bash 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone https://github.com/yuokamoto/io_use_case
cd ~/catkin_ws/
sudo rosdep init
rosdep update
sudo apt-get update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y -r
catkin_make
```

When you execute following command, you can see live video streaming at `<ip address of robot>:8080` even if you get few error message which come from minio client.
```
roslaunch img_uploader img_uploader.launch use_camera:=true use_web_video_server:=true
```

### Who do I talk to? ###

* Yu Okamoto (yuokamoto1988@gmail.com)