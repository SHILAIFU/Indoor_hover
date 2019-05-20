# 基于双目室内悬停飞行
    本项目实现无人机室内悬停飞行测试。
# 相关硬件
  
    1、M100无人机（参见<a href="https://www.dji.com/cn/matrice100/info>）
    2、TX2处理器（参见https://www.nvidia.cn/autonomous-machines/embedded-systems/jetson-tx2/）
    3、ZED双目摄像头(参见https://www.stereolabs.com/zed/)
# 软件安装
  
    1、TX2中ubuntu系统安装（参见）
    2、ROS安装（参见http://wiki.ros.org/ROS/Installation）
    3、DJI Onboard SDK 安装（参见https://developer.dji.com/onboard-sdk/）
    4、ZED ROS驱动安装（参见https://www.stereolabs.com/docs/ros/）
# 使用教程
    mkdir ~/catkin_ws/src/
    cd ~/catkin_ws/src/
    git clone https://github.com/tsrjrc/Indoor_hover.git
    cd ..
    catkin_make
