moveit assistant 、libmoveit.so库版本匹配问题解决：回退版本。

```bash
wget http://snapshots.ros.org/humble/2025-06-18/ubuntu/pool/main/r/ros-humble-rviz-common/ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb

http://ec2-35-174-230-37.compute-1.amazonaws.com/humble/2024-12-05/ubuntu/pool/main/r/ros-humble-moveit-visual-tools/


sudo dpkg -i ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb
```




# build
```
colcon build --cmake-args -DBUILD_TESTING=False --symlink-install
```