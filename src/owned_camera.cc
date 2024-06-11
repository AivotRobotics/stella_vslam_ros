#include "owned_camera.h"

#include <iostream>
#include <fstream>

#include <opencv2/imgcodecs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/integral_image_normal.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/load_map.hpp>

namespace owned_camera {

// Copied from Messages.h::CallService
template <class SV>
static bool CallService (
    const std::string & serviceName,
    const typename SV::Request::SharedPtr request,
    typename SV::Response::SharedPtr& response
    )
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger(__func__), "Call service " << serviceName);
    
    std::shared_ptr<rclcpp::Node> n = rclcpp::Node::make_shared("service_client");
    auto client = n->create_client<SV>(serviceName);

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger(__func__), "Interrupted while waiting for service %s. Exiting.", serviceName.c_str());
            throw std::runtime_error("Interrupted while waiting for service");
        }
        RCLCPP_INFO(rclcpp::get_logger(__func__), "service %s not available, waiting again...", serviceName.c_str());
    }

    auto result = client->async_send_request(request);
    bool isSuccess = (rclcpp::spin_until_future_complete(n, result) == rclcpp::FutureReturnCode::SUCCESS);
    if (! isSuccess) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(__func__), "Failed to call service " << serviceName);
    } else {
        response = result.get();
    }

    return isSuccess;
}

// Copied from Volume.cpp::CalcFramePosMap
void CalcPosMap (const Eigen::Affine3d & camPose, const rs2_intrinsics & cIntr, const cv::Mat & depthMap, cv::Mat & posMap) {
    posMap = cv::Mat (depthMap.rows, depthMap.cols, CV_32FC3, cv::Scalar (NAN,NAN,NAN));
    
    for (int row = 0; row < depthMap.rows; ++ row) {
        for (int col = 0; col < depthMap.cols; ++ col) {
            unsigned short d = depthMap.at <unsigned short> (row, col);
            if (d > 0) {
                double vz = (double) d / 1000.0f;
                Eigen::Vector3d pt = camPose * Eigen::Vector3d (vz, -(col - cIntr.ppx) * vz / cIntr.fx, -(row - cIntr.ppy) * vz/ cIntr.fy);
                posMap.at <cv::Vec3f> (row, col) = cv::Vec3f (pt[0], pt[1], pt[2]);
            }
        }
    }
}

// Copied from Frame.cpp Frame::ProcessDepth
std::vector <cv::Vec3f> ProcessDepth (const Eigen::Affine3d & camPose, const rs2_intrinsics & cIntr, const cv::Mat & depthMap) {
    const int numRows = depthMap.rows;
    const int numCols = depthMap.cols;
    pcl::PointCloud <pcl::PointXYZ>::Ptr vertexCloud (new pcl::PointCloud <pcl::PointXYZ> ());
    pcl::PointCloud <pcl::Normal>::Ptr normalCloud (new pcl::PointCloud <pcl::Normal>);
    cv::Mat posMap;
    std::vector <cv::Vec3f> resPts;
    
    CalcPosMap (camPose, cIntr, depthMap, posMap);

    vertexCloud->points.resize (numRows * numCols);
    vertexCloud->height = numRows;
    vertexCloud->width = numCols;
    vertexCloud->is_dense = false;

    for (int iter = 0; iter < numRows * numCols; ++ iter) {
        cv::Vec3f pt = posMap.at <cv::Vec3f> (iter/numCols, iter%numCols);
        pcl::PointXYZ pclPt;
        pclPt.x = pt[0];
        pclPt.y = pt[1];
        pclPt.z = pt[2];
        vertexCloud->points [iter] = pclPt;
    }

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> neCopy;
    neCopy.setNormalEstimationMethod(neCopy.AVERAGE_3D_GRADIENT);
    neCopy.setMaxDepthChangeFactor(0.008);
    neCopy.setNormalSmoothingSize(10.0);
    neCopy.setDepthDependentSmoothing(true);
    neCopy.setInputCloud(vertexCloud);
    neCopy.compute(*normalCloud);

    for (int iter = 0; iter < numRows * numCols; ++ iter) {
        const pcl::Normal & nPt = normalCloud->points [iter];
        if (!std::isnan (nPt.normal_x) && !std::isnan (nPt.normal_y) && ! std::isnan (nPt.normal_z)) {
            resPts.push_back (posMap.at <cv::Vec3f> (iter/numCols, iter%numCols));
        }
    }
    return resPts;
}

void WriteLiveMapYaml (const std::string & yamlFile, const std::string & mapFile) {
    std::ofstream file (yamlFile);
    file << "image: " << mapFile << std::endl;
    file << "resolution: 0.1" << std::endl;
    file << "origin: [-50.0, -50.0, 0]" << std::endl;
    file << "occupied_thresh: 0.65" << std::endl;
    file << "free_thresh: 0.196" << std::endl;
    file << "negate: 0" << std::endl;
}

void WriteLiveMap () {
    cv::Mat liveMap (1000, 1000, CV_8UC1, cv::Scalar (0));
    for (int c = 0; c < 1000; ++ c) {
        liveMap.at <unsigned char> (10, c) = 255;
        liveMap.at <unsigned char> (990, c) = 255;
        liveMap.at <unsigned char> (c, 10) = 255;
        liveMap.at <unsigned char> (c, 990) = 255;
    }

    std::string mapFile ("/WorkingData/live_map.pgm");
    std::string yamlFile ("/WorkingData/live_map.yaml");
    cv::imwrite (mapFile, liveMap);

    WriteLiveMapYaml (yamlFile, mapFile);
}
    
void ProcPoints(Eigen::Affine3d camPose, rs2_intrinsics cIntr, cv::Mat depthMap) {
    std::vector <cv::Vec3f> pts = ProcessDepth (camPose, cIntr, depthMap);
    
    WriteLiveMap();

    auto loadMapReq = std::make_shared<nav_msgs::srv::LoadMap::Request>();
    loadMapReq->map_url = "/WorkingData/live_map.yaml";
    nav_msgs::srv::LoadMap::Response::SharedPtr loadMapResp;
    CallService <nav_msgs::srv::LoadMap> ("/spectra/map_server/load_map", loadMapReq, loadMapResp);

}

}
