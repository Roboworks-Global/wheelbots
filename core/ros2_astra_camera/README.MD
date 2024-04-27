将wheeltec_ros2/src/ros2_astra_camera-main删除，替换成ros2_astra_camera功能包
step1:
```bash
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-humble-image-transport ros-galactic-image-publisher
```

step2:

```bash
wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz  -O glog-0.6.0.tar.gz
tar -xzvf glog-0.6.0.tar.gz
cd glog-0.6.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig  # Refreshing the link library
```

step3:

```bash
wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz

tar -xzvf magic_enum-0.8.0.tar.gz
cd magic_enum-0.8.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # Refreshing the link library
```

step4:

```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # Refreshing the link library
```

step5:

```bash
cd ~/wheeltec_ros2/src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

step6:

```bash
 cd ~/wheeltec_ws
source /opt/ros/humble/setup.bash 
colcon build --packages-select astra_camera_msgs 
colcon build --packages-select astra_camera
```



Starting the camera


```bash
For Astra S:
source /opt/ros/galactic/setup.bash
source ./install/setup.bash
ros2 launch astra_camera astra_mini.launch.py
```
