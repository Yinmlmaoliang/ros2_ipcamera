# 三相机ROS2组件系统使用说明

## 概述

此ROS2包实现了一个三相机系统，使用组件（Composition）架构在单个容器中运行三个相机节点，实现高效的资源共享和模块化设计。

## 主要特性

- **三相机支持**：在单个component_container中同时运行三个相机节点
- **简化配置**：通过YAML参数文件统一配置所有相机
- **去畸变支持**：可选的图像去畸变功能，三个相机共用同一个内参配置
- **纯图像发布**：只发布图像话题，不发布camera_info
- **无认证逻辑**：移除了独立的用户名密码参数，如需认证请在URL中嵌入凭据

## 配置说明

### 1. 编辑参数文件

修改 `config/ipcamera.yaml` 配置您的三个相机：

```yaml
camera1:
  ros__parameters:
    rtsp_url: "rtsp://192.168.1.100:554/stream1"    # 或 "rtsp://user:pass@ip:port/stream"
    image_width: 1920
    image_height: 1080
    enable_undistort: false

camera2:
  ros__parameters:
    rtsp_url: "rtsp://192.168.1.101:554/stream1"
    image_width: 1920
    image_height: 1080
    enable_undistort: false

camera3:
  ros__parameters:
    rtsp_url: "rtsp://192.168.1.102:554/stream1"
    image_width: 1920
    image_height: 1080
    enable_undistort: false
```

**RTSP URL格式说明**：
- 无认证：`rtsp://ip:port/stream`
- 有认证：`rtsp://username:password@ip:port/stream`
- 示例：`rtsp://admin:password123@192.168.1.100:554/stream1`

### 2. 相机内参配置（可选）

如果启用去畸变（`enable_undistort: true`），需要配置 `config/camera_info.yaml`：

```yaml
image_width: 1920
image_height: 1080
camera_name: "camera"
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
```

**注意**：三个相机将共用同一个内参配置文件。

## 编译与运行

### 编译

```bash
cd /path/to/your/workspace
colcon build --packages-select ros2_ipcamera --symlink-install
source install/setup.bash
```

### 运行

```bash
ros2 launch ros2_ipcamera ipcamera.launch.py
```

## 发布的话题

系统将发布以下三个图像话题：

- `/camera1/image_raw` (sensor_msgs/Image)
- `/camera2/image_raw` (sensor_msgs/Image)
- `/camera3/image_raw` (sensor_msgs/Image)

## 架构说明

### 组件架构

系统使用ROS2 Composition架构：
- **单容器**：`multi_camera_container`
- **三个组件节点**：camera1, camera2, camera3
- **共享进程**：所有相机在同一进程中运行，提高效率
- **模块化**：每个相机独立配置和运行

### 系统流程

1. Launch文件从YAML加载三个相机的参数
2. 创建一个component_container
3. 在容器中实例化三个IpCamera组件
4. 每个组件独立连接RTSP流并发布图像

## 查看话题

查看所有图像话题：
```bash
ros2 topic list | grep image_raw
```

查看特定相机的话题信息：
```bash
ros2 topic info /camera1/image_raw
ros2 topic echo /camera1/image_raw --no-arr
```

使用RQT查看图像：
```bash
ros2 run rqt_image_view rqt_image_view
```

## 常见问题

### 1. 相机连接失败（401 Unauthorized）

如果看到 "401 Unauthorized" 错误，说明相机需要认证。请在RTSP URL中嵌入用户名和密码：

```yaml
rtsp_url: "rtsp://admin:yourpassword@192.168.1.100:554/stream1"
```

### 2. 网络连接问题（No route to host）

确保：
- 相机IP地址正确
- 相机和运行节点的设备在同一网络
- 防火墙未阻止RTSP端口（通常是554）

### 3. 分辨率不匹配

节点不会调整图像大小，请确保配置的 `image_width` 和 `image_height` 与相机实际输出分辨率一致。如果不确定，节点会在日志中提示实际分辨率。

### 4. 去畸变失败

如果启用去畸变但失败，请检查：
- `config/camera_info.yaml` 文件存在
- 内参矩阵和畸变系数格式正确
- 配置的分辨率与实际图像一致

## 性能优化

- **QoS设置**：默认使用Best-effort + KeepLast(1)，适合实时流
- **发布频率**：约30Hz（30ms循环）
- **资源共享**：单容器架构减少内存开销

## 扩展说明

如需添加更多相机，需要：
1. 在 `config/ipcamera.yaml` 添加新的相机配置块
2. 在 `launch/ipcamera.launch.py` 添加对应的ComposableNode
3. 重新编译和启动

## 相关文件

- `src/ipcamera_component.cpp` - 相机组件实现
- `include/ros2_ipcamera/ipcamera_component.hpp` - 相机组件头文件
- `launch/ipcamera.launch.py` - 三相机launch文件
- `config/ipcamera.yaml` - 三相机参数配置
- `config/camera_info.yaml` - 相机内参配置（用于去畸变）
