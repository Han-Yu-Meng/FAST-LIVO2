/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include <fins/node.hpp>
#include <memory>
#include <mutex>
#include <thread>

#include "LIVMapper.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class FastLIVONode : public fins::Node {
public:
  void define() override {
    set_name("FastLIVO2");
    set_description("Fast Direct LiDAR-Inertial-Visual Odometry (FINS Port)");
    set_category("SLAM");

    register_input<0, livox_ros_driver2::msg::CustomMsg>(
        "lidar", &FastLIVONode::on_livox);

    register_input<1, sensor_msgs::msg::Imu>("imu", &FastLIVONode::on_imu);

    register_input<2, sensor_msgs::msg::Image>("image",
                                               &FastLIVONode::on_image);

    register_output<0, sensor_msgs::msg::PointCloud2>("cloud_registered");
    register_output<1, nav_msgs::msg::Path>("path");
    register_output<2, nav_msgs::msg::Odometry>("odometry");
    register_output<3, geometry_msgs::msg::TransformStamped>("transform");
  }

  void initialize() override {
    logger->info("Initializing FastLIVO2 Node...");

    mapper_ = std::make_shared<LIVMapper>(this);

    is_running_ = true;

    logger->info("FastLIVO2 Node initialized.");
  }

  void deinitialize() {
    is_running_ = false;
    mapper_.reset();
  }

  ~FastLIVONode() { deinitialize(); }

  void run() override {}

  void pause() override {}
  void reset() override {}

  void on_lidar(const fins::Msg<sensor_msgs::msg::PointCloud2> &msg) {
    if (mapper_) {
      mapper_->push_lidar(msg.data);
      mapper_->trigger();
    }
  }

  void on_livox(const fins::Msg<livox_ros_driver2::msg::CustomMsg> &msg) {
    if (mapper_) {
      mapper_->push_livox(msg.data);
      mapper_->trigger();
    }
  }

  void on_imu(const fins::Msg<sensor_msgs::msg::Imu> &msg) {
    if (mapper_) {
      mapper_->push_imu(msg.data);
      mapper_->trigger();
    }
  }

  void on_image(const fins::Msg<sensor_msgs::msg::Image> &msg) {
    if (mapper_) {
      mapper_->push_img(msg.data);
      mapper_->trigger();
    }
  }

private:
  std::shared_ptr<LIVMapper> mapper_;
  bool is_running_ = false;
};

EXPORT_NODE(FastLIVONode)