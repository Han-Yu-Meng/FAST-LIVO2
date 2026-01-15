/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include <atomic>
#include <condition_variable>
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
    has_new_data_ = false;
    mapping_thread_ = std::thread(&FastLIVONode::mapping_worker_loop, this);

    logger->info("FastLIVO2 Node initialized with independent mapping thread.");
  }

  void deinitialize() {
    is_running_ = false;
    trigger_cv_.notify_all();

    if (mapping_thread_.joinable()) {
      mapping_thread_.join();
    }

    mapper_.reset();
  }

  ~FastLIVONode() { deinitialize(); }

  void run() override {}
  void pause() override {}
  void reset() override {}

  void on_lidar(const fins::Msg<sensor_msgs::msg::PointCloud2> &msg) {
    if (mapper_) {
      FINS_TIME_BLOCK(logger, "Lidar Callback");
      mapper_->push_lidar(msg.data);
    }
  }

  void on_livox(const fins::Msg<livox_ros_driver2::msg::CustomMsg> &msg) {
    if (mapper_) {
      FINS_TIME_BLOCK(logger, "Livox Callback");
      mapper_->push_livox(msg.data);
    }
  }

  void on_imu(const fins::Msg<sensor_msgs::msg::Imu> &msg) {
    if (mapper_) {
      FINS_TIME_BLOCK(logger, "IMU Callback");
      mapper_->push_imu(msg.data);
    }
  }

  void on_image(const fins::Msg<sensor_msgs::msg::Image> &msg) {
    if (mapper_) {
      FINS_TIME_BLOCK(logger, "Image Callback");
      mapper_->push_img(msg.data);
    }
  }

private:
  void notify_backend() {
    {
      std::lock_guard<std::mutex> lock(trigger_mtx_);
      has_new_data_ = true;
    }
    trigger_cv_.notify_one();
  }

  void mapping_worker_loop() {
    logger->info("Backend mapping worker started (Timer Mode).");

    while (is_running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      if (!is_running_)
        break;

      if (mapper_) {
        // FINS_TIME_BLOCK(logger, "LIVMapper Trigger");
        mapper_->trigger();
      }
    }
    logger->info("Backend mapping worker exiting.");
  }

  std::shared_ptr<LIVMapper> mapper_;

  std::thread mapping_thread_;
  std::mutex trigger_mtx_;
  std::condition_variable trigger_cv_;
  std::atomic<bool> is_running_{false};
  bool has_new_data_{false};
};

EXPORT_NODE(FastLIVONode)