// A one-shot node that subscribes to camera intrinsics (CameraInfo) to generate a Stella VSLAM YAML config.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <string>
#include <filesystem>

using namespace std::chrono_literals;

class ConfigWriter : public rclcpp::Node {
public:
  ConfigWriter() : Node("config_writer") {
    camera_info_topic_ = declare_parameter<std::string>(
        "camera_info_topic", "camnav/zed/rgb/camera_info");
    robot_name_ = declare_parameter<std::string>("robot_name", "");
    output_path_ = declare_parameter<std::string>(
        "output_path", "");
    camera_setup_ = declare_parameter<std::string>("camera_setup", "RGBD");
    color_order_ = declare_parameter<std::string>("color_order", "RGB");
    fps_ = declare_parameter<double>("fps", 30.0);
    min_size_ = declare_parameter<int>("min_size", 800);
    assume_rectified_ = declare_parameter<bool>("assume_rectified", true);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 5.0);

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          if (!cam_info_) cam_info_ = msg;
        });

    timer_ = create_wall_timer(200ms, std::bind(&ConfigWriter::tick, this));

    auto to_abs = [this](const std::string & topic) {
      if (!topic.empty() && topic.front() == '/') {
        return topic;
      }
      std::string ns = this->get_namespace();
      if (ns.empty() || ns == "/") {
        return std::string("/") + topic;
      }
      if (ns.back() == '/') {
        return ns + topic;
      }
      return ns + "/" + topic;
    };

    RCLCPP_INFO(get_logger(), "Waiting for CameraInfo on: %s", to_abs(camera_info_topic_).c_str());
  }

private:
  void tick() {
    auto now = this->now();
    if (!start_time_set_) {
      start_time_ = now;
      start_time_set_ = true;
    }

    if (cam_info_) {
      try {
        write_yaml();
        rclcpp::shutdown();
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Failed to write YAML: %s", e.what());
        rclcpp::shutdown();
      }
      return;
    }

    double elapsed = (now - start_time_).seconds();
    if (elapsed > timeout_sec_) {
      RCLCPP_ERROR(get_logger(), "Timeout (%.1fs). Missing: %s%s", elapsed,
                   cam_info_ ? "" : "CameraInfo ");
      rclcpp::shutdown();
    }
  }

  void write_yaml() {
    const auto &ci = *cam_info_;

    // Intrinsics
    double fx = ci.k[0];
    double fy = ci.k[4];
    double cx = ci.k[2];
    double cy = ci.k[5];
    int cols = static_cast<int>(ci.width);
    int rows = static_cast<int>(ci.height);

    // Distortion
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;
    if (!assume_rectified_ && ci.distortion_model == "plumb_bob" && ci.d.size() >= 5) {
      k1 = ci.d[0];
      k2 = ci.d[1];
      p1 = ci.d[2];
      p2 = ci.d[3];
      k3 = ci.d[4];
    }

    YAML::Node root;

    auto cam = root["Camera"];
    cam["name"] = ci.header.frame_id.empty() ? std::string("ZED") : ci.header.frame_id;
    cam["setup"] = camera_setup_;
    cam["model"] = "perspective";
    cam["fx"] = fx;
    cam["fy"] = fy;
    cam["cx"] = cx;
    cam["cy"] = cy;
    cam["k1"] = k1;
    cam["k2"] = k2;
    cam["p1"] = p1;
    cam["p2"] = p2;
    cam["k3"] = k3;
    cam["fps"] = fps_;
    cam["cols"] = cols;
    cam["rows"] = rows;
    cam["focal_x_baseline"] = 40.0;
    cam["depth_threshold"] = 40.0;
    cam["color_order"] = color_order_;

    auto prep = root["Preprocessing"];
    prep["min_size"] = min_size_;
    prep["depthmap_factor"] =  1.0; // default for 32FC1 (meters)
    prep["descriptor_type"] = "HashSIFT";

    auto feat = root["Feature"];
    feat["scale_factor"] = 1.2;
    feat["num_levels"] = 8;
    feat["ini_fast_threshold"] = 20;
    feat["min_fast_threshold"] = 7;

    auto mapping = root["Mapping"];
    mapping["baseline_dist_thr"] = 0.07471049682;
    mapping["redundant_obs_ratio_thr"] = 0.9;

    auto pangolin = root["PangolinViewer"];
    pangolin["keyframe_size"] = 0.05;
    pangolin["keyframe_line_width"] = 1;
    pangolin["graph_line_width"] = 1;
    pangolin["point_size"] = 2;
    pangolin["camera_size"] = 0.08;
    pangolin["camera_line_width"] = 3;
    pangolin["viewpoint_x"] = 0;
    pangolin["viewpoint_y"] = -0.9;
    pangolin["viewpoint_z"] = -1.9;

    std::string out_path = output_path_;
    if (out_path.empty()) {
      out_path = "/WorkingData/stella-slam/" + robot_name_ + "/stella_camnav_zed.yaml";
    }

    std::filesystem::path outp(out_path);
    std::error_code ec;
    std::filesystem::create_directories(outp.parent_path(), ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Could not ensure directories: %s", ec.message().c_str());
    }

    std::ofstream ofs(out_path);
    if (!ofs.is_open()) {
      throw std::runtime_error("Cannot open output path: " + out_path);
    }
    ofs << root;
    ofs.close();

    RCLCPP_INFO(get_logger(), "Wrote Stella config to: %s", out_path.c_str());
  }

  std::string camera_info_topic_;
  std::string robot_name_;
  std::string output_path_;
  std::string camera_setup_;
  std::string color_order_;
  double fps_{};
  int min_size_{};
  bool assume_rectified_{};
  double timeout_sec_{};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_{};
  bool start_time_set_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConfigWriter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
