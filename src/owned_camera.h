#pragma once

#include <opencv2/core/core.hpp>
#include <librealsense2/rs.hpp>
#include <unsupported/Eigen/EulerAngles>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

namespace owned_camera {

void ProcPoints (const Eigen::Affine3d & deltaCamPose, const rs2_intrinsics & cIntr, const cv::Mat & depthMap, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> & points_pub, const rclcpp::Time & nodeTime, const std::string & frame_id);

}
