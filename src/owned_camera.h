#pragma once

#include <opencv2/core/core.hpp>
#include <librealsense2/rs.hpp>
#include <unsupported/Eigen/EulerAngles>

namespace owned_camera {

void ProcPoints (Eigen::Affine3d camPose, rs2_intrinsics cIntr, cv::Mat depthMap);

}
