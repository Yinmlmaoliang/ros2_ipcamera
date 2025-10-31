// Copyright (c) 2019 Tasuku Miura
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef IPCAMERA_COMPONENT_H
#define IPCAMERA_COMPONENT_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "ros2_ipcamera/visibility_control.hpp"
#include <image_transport/image_transport.hpp>
#include <chrono>


using namespace std::chrono_literals;

namespace ros2_ipcamera
{
  class IpCamera : public rclcpp::Node
  {
  public:
    /**
     * Instantiates the IpCamera Node.
     */
    COMPOSITION_PUBLIC
    explicit IpCamera(const std::string& node_name, const rclcpp::NodeOptions & options);

    /**
     * Delegates construction.
     */
    COMPOSITION_PUBLIC
    explicit IpCamera(const rclcpp::NodeOptions & options);

    /**;
     * Configures component.
     *
     * Declares parameters and configures video capture.
     */
    COMPOSITION_PUBLIC
    void
    configure();

    /**;
     * Declares the parameter using rcl_interfaces.
     */
    COMPOSITION_PUBLIC
    void
    initialize_parameters();

    /**;
     * Captures frame and converts frame to message.
     */
    COMPOSITION_PUBLIC
    void
    execute();

  private:
    image_transport::Publisher pub_;
    rclcpp::QoS qos_;
    std::chrono::milliseconds freq_ = 30ms;

    cv::VideoCapture cap_;
    std::string rtsp_url_;
    std::string rtsp_username_;
    std::string rtsp_password_;
    std::string source_;
    int width_;
    int height_;

    // Undistortion parameters
    bool enable_undistort_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat map1_;
    cv::Mat map2_;

    std::string
    mat_type2encoding(int mat_type);

    void
    convert_frame_to_message(
      const cv::Mat & frame,
      size_t frame_id,
      sensor_msgs::msg::Image & msg);

    void
    initialize_undistort_maps();
  };
}  // namespace ros2_ipcamera

#endif // IPCAMERA_COMPONENT_H
