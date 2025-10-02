# TASK04

## 安装依赖
确保C++,ROS2,MVS SDK,等依赖已经安装

## 命令行编译
```bash
colcon build --packages-select hik_camera
source install/setup.bash
```

## 运行节点
```bash
ros2 launch hik_camera hik_camera.launch.py
```
## 配置参数
编辑 `config/params.yaml` 文件修改相机参数：
- camera_ip: 相机IP地址
- exposure_time: 曝光时间
- gain: 增益值
- frame_rate: 帧率
- pixel_format: 像素格式(BGR8/RGB8/Mono8)
- image_topic: 图像话题名称
