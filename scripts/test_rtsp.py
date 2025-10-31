#!/usr/bin/env python3
import rclpy
import numpy as np
import cv2
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RTSPUndistortPublisher(Node):
    def __init__(self):
        super().__init__('rtsp_undistort_publisher')
        
        # 参数声明与获取
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rtsp_url', 'rtsp://192.168.8.30/left'),
                ('output_topic', '/camera/image_raw'),
                ('frame_rate', 30),
                ('reconnect_interval', 5.0)
            ]
        )
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        # 初始化视频流
        self.cap = None
        self._init_video_capture()
        self.frame_size = (1920, 1056)  # 默认分辨率，后续自动更新

        # 相机标定参数（根据实际设备修改）
        self.camera_matrix = np.array([
            [702.245153, 0.0, 934.261322],
            [0.0, 708.097554, 503.348681],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([
            [-0.262244, 0.047490, 0.000767, 0.002689, 0.0]
        ], dtype=np.float32)

        # 初始化ROS组件
        self.bridge = CvBridge()
        self._init_publisher()
        self._init_undistort_maps()
        self._init_timers()

        self.get_logger().info("节点初始化完成")

    def _init_video_capture(self):
        """初始化视频捕获并验证有效性"""
        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            self.get_logger().error("初始RTSP连接失败")
            raise RuntimeError("RTSP连接失败")
        
        # 验证首帧获取
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error("无法获取初始视频帧")
            raise RuntimeError("视频帧获取失败")
        
        self.frame_size = (frame.shape[1], frame.shape[0])
        self.get_logger().info(f"初始分辨率: {self.frame_size[0]}x{self.frame_size[1]}")

    def _init_publisher(self):
        """初始化发布器"""
        self.publisher = self.create_publisher(
            Image, 
            self.output_topic, 
            10  # 保持队列深度为10的默认设置
        )

    def _init_undistort_maps(self):
        """初始化去畸变映射表"""
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            self.camera_matrix,
            self.frame_size,
            cv2.CV_16SC2
        )
        self.get_logger().debug("去畸变映射表生成完成")

    def _init_timers(self):
        """初始化定时器系统"""
        # 主处理定时器
        self.create_timer(1.0 / self.frame_rate, self._frame_callback)
        # 健康监测定时器
        self.create_timer(5.0, self._health_check)

    def _frame_callback(self):
        """主处理回调函数"""
        try:
            # 帧捕获
            ret, frame = self.cap.read()
            if not self._validate_frame(ret, frame):
                return

            # 分辨率变更检测
            if (frame.shape[1], frame.shape[0]) != self.frame_size:
                self._handle_resolution_change(frame)

            # 去畸变处理
            undistorted = self._process_undistort(frame)
            if undistorted is None:
                return

            # 消息发布
            self._publish_frame(undistorted)

        except Exception as e:
            self.get_logger().error(f"处理流程异常: {str(e)}", throttle_duration_sec=5)

    def _validate_frame(self, ret, frame):
        """验证视频帧有效性"""
        if not ret:
            self.get_logger().warn("视频帧获取失败，尝试重新连接...", throttle_duration_sec=5)
            self._safe_reconnect()
            return False
        if frame is None or frame.size == 0:
            self.get_logger().warn("收到空帧", throttle_duration_sec=5)
            return False
        if frame.dtype != np.uint8:
            self.get_logger().warn(f"异常数据类型: {frame.dtype}", throttle_duration_sec=5)
            return False
        return True

    def _handle_resolution_change(self, frame):
        """处理分辨率变更"""
        new_size = (frame.shape[1], frame.shape[0])
        self.get_logger().warning(f"分辨率变更: {self.frame_size} -> {new_size}")
        self.frame_size = new_size
        self._init_undistort_maps()

    def _process_undistort(self, frame):
        """执行去畸变处理"""
        try:
            undistorted = cv2.remap(
                frame, 
                self.map1, 
                self.map2, 
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT
            )
            if undistorted.shape != frame.shape:
                self.get_logger().error(f"去畸变后尺寸异常: {undistorted.shape}")
                return Nonecamera_frame
            return undistorted
        except Exception as e:
            self.get_logger().error(f"去畸变处理失败: {str(e)}")
            return None

    def _publish_frame(self, frame):
        """发布图像消息"""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.publisher.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

    def _safe_reconnect(self):
        """安全的重新连接流程"""
        self.cap.release()
        time.sleep(self.reconnect_interval)
        self._init_video_capture()
        self.get_logger().info("重新连接成功")

    def _health_check(self):
        """系统健康状态检查"""
        if not self.cap.isOpened():
            self.get_logger().error("视频流连接丢失")
            self._safe_reconnect()

    def __del__(self):
        """资源清理"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        node = RTSPUndistortPublisher()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"致命错误: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()