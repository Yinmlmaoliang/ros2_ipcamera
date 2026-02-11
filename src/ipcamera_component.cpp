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
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_ipcamera/ipcamera_component.hpp"


namespace ros2_ipcamera
{
  IpCamera::IpCamera(const std::string & node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options),
    qos_(rclcpp::QoS(rclcpp::KeepLast(1)).best_effort())
  {
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    RCLCPP_INFO(this->get_logger(), "name: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "middleware: %s", rmw_get_implementation_identifier());

    // Declare parameters.
    this->initialize_parameters();

    this->configure();

    //TODO(Tasuku): add call back to handle parameter events.
    // Set up publishers.
    this->pub_ = image_transport::create_publisher(
      this, "~/image_raw", qos_.get_rmw_qos_profile());

    // Create timer for frame capture instead of blocking loop
    this->timer_ = this->create_wall_timer(
      this->freq_,
      std::bind(&IpCamera::timer_callback, this));
  }

  IpCamera::IpCamera(const rclcpp::NodeOptions & options)
  : IpCamera::IpCamera("ipcamera", options)
  {}

  void
  IpCamera::configure()
  {
    rclcpp::Logger node_logger = this->get_logger();

    // Get RTSP URL parameter
    this->get_parameter<std::string>("rtsp_url", rtsp_url_);
    RCLCPP_INFO(node_logger, "RTSP URL: %s", rtsp_url_.c_str());
    RCLCPP_INFO(node_logger, "Connecting to RTSP stream...");

    this->get_parameter<int>("image_width", width_);
    RCLCPP_INFO(node_logger, "image_width: %d", width_);

    this->get_parameter<int>("image_height", height_);
    RCLCPP_INFO(node_logger, "image_height: %d", height_);

    // Get undistortion parameter
    this->get_parameter<bool>("enable_undistort", enable_undistort_);
    RCLCPP_INFO(node_logger, "enable_undistort: %s", enable_undistort_ ? "true" : "false");

    // Load camera calibration parameters if undistortion is enabled
    if (enable_undistort_) {
      // Load camera_info.yaml to get calibration parameters
      std::string package_share_dir = ament_index_cpp::get_package_share_directory("ros2_ipcamera");
      std::string camera_info_file = package_share_dir + "/config/camera_info.yaml";

      RCLCPP_INFO(node_logger, "Loading camera calibration from: %s", camera_info_file.c_str());

      std::ifstream file(camera_info_file);
      if (!file.is_open()) {
        RCLCPP_ERROR(node_logger, "Failed to open camera_info.yaml");
        throw std::runtime_error("Failed to open camera_info.yaml");
      }

      // Parse YAML to extract camera matrix and distortion coefficients
      std::string line;
      std::vector<double> camera_matrix_data;
      std::vector<double> dist_coeffs_data;
      bool reading_camera_matrix = false;
      bool reading_dist_coeffs = false;

      while (std::getline(file, line)) {
        if (line.find("camera_matrix:") != std::string::npos) {
          reading_camera_matrix = true;
          reading_dist_coeffs = false;
        } else if (line.find("distortion_coefficients:") != std::string::npos) {
          reading_camera_matrix = false;
          reading_dist_coeffs = true;
        } else if (reading_camera_matrix && line.find("data:") != std::string::npos) {
          // Extract data array from line like "  data: [702.245153, 0.0, 934.261322, ...]"
          size_t start = line.find('[');
          size_t end = line.find(']');
          if (start != std::string::npos && end != std::string::npos) {
            std::string data_str = line.substr(start + 1, end - start - 1);
            std::stringstream ss(data_str);
            std::string token;
            while (std::getline(ss, token, ',')) {
              camera_matrix_data.push_back(std::stod(token));
            }
            reading_camera_matrix = false;
          }
        } else if (reading_dist_coeffs && line.find("data:") != std::string::npos) {
          size_t start = line.find('[');
          size_t end = line.find(']');
          if (start != std::string::npos && end != std::string::npos) {
            std::string data_str = line.substr(start + 1, end - start - 1);
            std::stringstream ss(data_str);
            std::string token;
            while (std::getline(ss, token, ',')) {
              dist_coeffs_data.push_back(std::stod(token));
            }
            reading_dist_coeffs = false;
          }
        }
      }
      file.close();

      // Validate data
      if (camera_matrix_data.size() != 9) {
        RCLCPP_ERROR(node_logger, "Invalid camera_matrix data size: %zu (expected 9)",
                     camera_matrix_data.size());
        throw std::runtime_error("Invalid camera_matrix in camera_info.yaml");
      }
      if (dist_coeffs_data.size() != 5) {
        RCLCPP_ERROR(node_logger, "Invalid distortion_coefficients data size: %zu (expected 5)",
                     dist_coeffs_data.size());
        throw std::runtime_error("Invalid distortion_coefficients in camera_info.yaml");
      }

      // Create OpenCV matrices
      camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
      dist_coeffs_ = cv::Mat(1, 5, CV_64F, dist_coeffs_data.data()).clone();

      RCLCPP_INFO(node_logger, "Camera calibration loaded successfully");

      // Initialize undistortion maps (will be called after video capture is opened)
    }

    // TODO(Tasuku): move to on_configure() when rclcpp_lifecycle available.
    this->cap_.open(rtsp_url_);

    if (!this->cap_.isOpened()) {
      RCLCPP_ERROR(node_logger, "Could not open video stream");
      throw std::runtime_error("Could not open video stream");
    }

    // Try to set the width and height based on parameters.
    // Note: For IP cameras, these set() calls may not be effective as cameras use fixed resolutions.
    // The actual resolution will be read from the stream below.
    this->cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    this->cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));

    // Read actual resolution from the video stream
    int actual_width = static_cast<int>(this->cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_height = static_cast<int>(this->cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

    RCLCPP_INFO(node_logger, "RTSP stream opened successfully!");

    if (actual_width > 0 && actual_height > 0) {
      // Check if actual resolution differs from configured
      if (actual_width != width_ || actual_height != height_) {
        RCLCPP_WARN(node_logger,
                    "Configured resolution (%dx%d) differs from actual stream resolution (%dx%d). Using actual resolution.",
                    width_, height_, actual_width, actual_height);
      }
      // Always use actual resolution from stream
      width_ = actual_width;
      height_ = actual_height;
      RCLCPP_INFO(node_logger, "Using stream resolution: %dx%d", width_, height_);
    } else {
      RCLCPP_WARN(node_logger,
                  "Could not read resolution from stream properties, using configured values: %dx%d",
                  width_, height_);
    }

    // Initialize undistortion maps if enabled (using actual resolution)
    if (enable_undistort_) {
      initialize_undistort_maps();
    }
  }

  void
  IpCamera::initialize_parameters()
  {
    rcl_interfaces::msg::ParameterDescriptor rtsp_url_descriptor;
    rtsp_url_descriptor.name = "rtsp_url";
    rtsp_url_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    rtsp_url_descriptor.description = "RTSP URL of the IP camera.";
    rtsp_url_descriptor.additional_constraints = "Should be of the form 'rtsp://ip:port/stream'";
    this->declare_parameter("rtsp_url", "", rtsp_url_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_width_descriptor;
    image_width_descriptor.name = "image_width";
    image_width_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("image_width", 640, image_width_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_height_descriptor;
    image_height_descriptor.name = "image_height";
    image_height_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("image_height", 480, image_height_descriptor);

    rcl_interfaces::msg::ParameterDescriptor enable_undistort_descriptor;
    enable_undistort_descriptor.name = "enable_undistort";
    enable_undistort_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    enable_undistort_descriptor.description = "Enable image undistortion using camera calibration parameters.";
    this->declare_parameter("enable_undistort", false, enable_undistort_descriptor);
  }

  void
  IpCamera::initialize_undistort_maps()
  {
    rclcpp::Logger node_logger = this->get_logger();

    if (camera_matrix_.empty() || dist_coeffs_.empty()) {
      RCLCPP_ERROR(node_logger, "Camera matrix or distortion coefficients not loaded");
      throw std::runtime_error("Camera calibration data not available");
    }

    cv::Size image_size(width_, height_);

    // Initialize undistortion maps using original camera matrix (no optimization)
    cv::initUndistortRectifyMap(
      camera_matrix_,
      dist_coeffs_,
      cv::Mat(),  // No rectification
      camera_matrix_,  // Use original camera matrix (no optimization)
      image_size,
      CV_16SC2,  // Map type for efficiency
      map1_,
      map2_
    );

    RCLCPP_INFO(node_logger, "Undistortion maps initialized for image size %dx%d",
                width_, height_);
  }

  void
  IpCamera::timer_callback()
  {
    // Initialize OpenCV image matrices.
    static cv::Mat frame;
    static cv::Mat undistorted_frame;
    static size_t frame_id = 0;

    // Initialize a shared pointer to an Image message.
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;

    // Get the frame from the video capture.
    this->cap_ >> frame;
    // Check if the frame was grabbed correctly
    if (!frame.empty()) {
      // Apply undistortion if enabled
      if (enable_undistort_) {
        cv::remap(frame, undistorted_frame, map1_, map2_, cv::INTER_LINEAR);
        // Convert undistorted frame to a ROS image
        convert_frame_to_message(undistorted_frame, frame_id, *msg);
      } else {
        // Convert original frame to a ROS image
        convert_frame_to_message(frame, frame_id, *msg);
      }
      // Publish the image message and increment the frame_id.
      this->pub_.publish(std::move(msg));
      ++frame_id;
    }
  }

  void
  IpCamera::execute()
  {
    // Deprecated: This method is kept for backward compatibility
    // but is no longer used. Frame capture is now handled by timer_callback().
    RCLCPP_WARN(this->get_logger(),
                "execute() is deprecated and should not be called directly. "
                "Frame capture is now handled by timer callbacks.");
  }

  std::string
  IpCamera::mat_type2encoding(int mat_type)
  {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("Unsupported encoding type");
    }
  }

  void
  IpCamera::convert_frame_to_message(
    const cv::Mat & frame,
    size_t /* frame_id */,
    sensor_msgs::msg::Image & msg)
  {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);

    rclcpp::Time timestamp = this->get_clock()->now();

    msg.header.frame_id = "camera_frame";
    msg.header.stamp = timestamp;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_ipcamera::IpCamera)
