#include "owned_camera.h"

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/integral_image_normal.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <nav_msgs/srv/load_map.hpp>

namespace owned_camera {

// Constant definitions.
// TODO: The camera pose is hardcoded instead of fetching from URDF / tf.
const Eigen::Vector3d baseCamPos (-0.0267, 0.2336, 0.7766);
const Eigen::Vector3d baseCamRot(0, 0.2618, 0);
const int MapNumRows = 1000;
const int MapNumCols = 1000;
const float MapVoxLen = 0.1;
const float MapLowRow = -50.0;
const float MapLowCol = -50.0;
const float MinObstacleHeight = 0.1;
const int MinVoxPts = 5;

// Global-Map stores the maximum height across all the points in the
// xy-grid. It maintains unknown regions as NaN value. Live-Map stores
// the occupancy grid (binary state for now).
//
cv::Mat globalMap (MapNumRows, MapNumCols, CV_32FC1, cv::Scalar(NAN));
cv::Mat liveMap (MapNumRows, MapNumCols, CV_8UC1, cv::Scalar (0));
    
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
void CalcPosMap (const rs2_intrinsics & cIntr, const cv::Mat & depthMap, cv::Mat & posMap) {
    posMap = cv::Mat (depthMap.rows, depthMap.cols, CV_32FC3, cv::Scalar (NAN,NAN,NAN));    
    for (int row = 0; row < depthMap.rows; ++ row) {
        for (int col = 0; col < depthMap.cols; ++ col) {
            unsigned short d = depthMap.at <unsigned short> (row, col);
            if (d > 0) {
                double vz = (double) d / 1000.0f;
                posMap.at <cv::Vec3f> (row, col) = cv::Vec3f (vz, -(col - cIntr.ppx) * vz / cIntr.fx, -(row - cIntr.ppy) * vz/ cIntr.fy);
            }
        }
    }
}

// Copied from Frame.cpp Frame::ProcessDepth
std::vector <cv::Vec3f> ProcessDepth (const rs2_intrinsics & cIntr, const cv::Mat & depthMap) {
    const int numRows = depthMap.rows;
    const int numCols = depthMap.cols;
    pcl::PointCloud <pcl::PointXYZ>::Ptr vertexCloud (new pcl::PointCloud <pcl::PointXYZ> ());
    pcl::PointCloud <pcl::Normal>::Ptr normalCloud (new pcl::PointCloud <pcl::Normal>);
    cv::Mat posMap;
    std::vector <cv::Vec3f> resPts;
    
    CalcPosMap (cIntr, depthMap, posMap);

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

void WriteLiveMap (const std::vector <cv::Vec3f> & pts) {
    cv::Mat currMap (MapNumRows, MapNumCols, CV_32FC1, cv::Scalar (NAN));
    cv::Mat currMapCount (MapNumRows, MapNumCols, CV_32SC1, cv::Scalar (0));

    for (const auto & pt : pts) {
        int r = (pt[1] - MapLowRow)/MapVoxLen;
        int c = (pt[0] - MapLowCol)/MapVoxLen;
        if (r < 0 || c < 0 || r >= MapNumRows || c >= MapNumCols) {
            continue;
        }
        
        ++ currMapCount.at <int> (r, c);

        bool hasCurrUpdate = (std::isnan (currMap.at <float> (r,c)) || currMap.at <float> (r,c) < pt[2]);
        bool hasGlobalUpdate = (currMapCount.at <int> (r, c) == MinVoxPts || (currMapCount.at <int> (r, c) > MinVoxPts && hasCurrUpdate));
        
        if (hasCurrUpdate) {
            currMap.at <float> (r, c) = pt[2];
        }

        if (hasGlobalUpdate) {
            globalMap.at <float> (r, c) = currMap.at <float> (r, c);
            liveMap.at <unsigned char> (r, c) = ((currMap.at <float> (r, c) < MinObstacleHeight)? 0 : 255);
        }
    }

    // cv::namedWindow ("map", cv::WindowFlags::WINDOW_AUTOSIZE);
    // cv::imshow ("map", liveMap);
    // cv::waitKey (1);

    std::string mapFile ("/WorkingData/live_map.pgm");
    std::string yamlFile ("/WorkingData/live_map.yaml");
    cv::imwrite (mapFile, liveMap);

    WriteLiveMapYaml (yamlFile, mapFile);
}

sensor_msgs::msg::PointCloud2 BuildPointsMsg (const std::vector <cv::Vec3f> & pts, const rclcpp::Time & nodeTime, const std::string & frame_id) {
    sensor_msgs::msg::PointCloud2 points_msg;
    
    sensor_msgs::PointCloud2Modifier modifier(points_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pts.size());

    points_msg.row_step = points_msg.width * points_msg.point_step;
    points_msg.data.resize(points_msg.height * points_msg.row_step);

    sensor_msgs::PointCloud2Iterator<float>iter_x(points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float>iter_y(points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float>iter_z(points_msg, "z");
    
    for (const auto & pt : pts) {
        *iter_x = pt[0];
        *iter_y = pt[1];
        *iter_z = pt[2];
        ++iter_x; ++iter_y; ++iter_z;
    }
    
    points_msg.header.stamp = nodeTime;
    points_msg.header.frame_id = frame_id;
    points_msg.is_dense = true;

    return points_msg;
}
    
void ProcPoints(const Eigen::Affine3d & deltaCamPose, const rs2_intrinsics & cIntr, const cv::Mat & depthMap, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> & points_pub, const rclcpp::Time & nodeTime, const std::string & frame_id) {

    Eigen::Affine3d baseCamPose (Eigen::Affine3d::Identity());
    baseCamPose.translation() = baseCamPos;
    baseCamPose.linear() = Eigen::EulerAngles<double, Eigen::EulerSystemZYX>(baseCamRot[2], baseCamRot[1], baseCamRot[0]).toRotationMatrix();
    Eigen::Affine3d camPose = deltaCamPose * baseCamPose;
    
    std::vector <cv::Vec3f> camPts = ProcessDepth (cIntr, depthMap);

    std::vector <cv::Vec3f> wPts;
    for (const auto & cPt : camPts) {
        Eigen::Vector3d wPt = camPose * Eigen::Vector3d (cPt[0], cPt[1], cPt[2]);
        wPts.push_back (cv::Vec3f (wPt[0], wPt[1], wPt[2]));
    }
    
    WriteLiveMap(wPts);

    auto loadMapReq = std::make_shared<nav_msgs::srv::LoadMap::Request>();
    loadMapReq->map_url = "/WorkingData/live_map.yaml";
    nav_msgs::srv::LoadMap::Response::SharedPtr loadMapResp;
    CallService <nav_msgs::srv::LoadMap> ("/husky/map_server/load_map", loadMapReq, loadMapResp);

    sensor_msgs::msg::PointCloud2 points_msg = BuildPointsMsg (camPts, nodeTime, frame_id);
    points_pub->publish(points_msg);
}

}
