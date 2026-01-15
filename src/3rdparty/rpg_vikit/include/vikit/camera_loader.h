/*
 * camera_loader.h
 *
 *  Adapted for FINS Framework
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <iostream>
#include <string>
#include <vector>

#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/polynomial_camera.h>

#include <fins/agent/parameter_server.hpp>
#include <fins/utils/logger.hpp>

namespace vk {
namespace camera_loader {

inline bool loadFromFins(const std::string &ns, vk::AbstractCamera *&cam) {
  fins::ParamLoader loader(ns);

  bool res = true;
  std::string cam_model = loader.load("cam_model", std::string(""));

  if (cam_model.empty()) {
    FINS_LOG_ERROR("[CameraLoader] 'cam_model' not found in namespace: {}", ns);
    return false;
  }

  if (cam_model == "Ocam") {
    std::string calib_file = loader.load("cam_calib_file", std::string(""));
    cam = new vk::OmniCamera(calib_file);
  } else if (cam_model == "Pinhole") {
    cam = new vk::PinholeCamera(
        loader.load("cam_width", 0), loader.load("cam_height", 0),
        loader.load("scale", 1.0), loader.load("cam_fx", 0.0),
        loader.load("cam_fy", 0.0), loader.load("cam_cx", 0.0),
        loader.load("cam_cy", 0.0), loader.load("cam_d0", 0.0),
        loader.load("cam_d1", 0.0), loader.load("cam_d2", 0.0),
        loader.load("cam_d3", 0.0));
  } else if (cam_model == "EquidistantCamera") {
    cam = new vk::EquidistantCamera(
        loader.load("cam_width", 0), loader.load("cam_height", 0),
        loader.load("scale", 1.0), loader.load("cam_fx", 0.0),
        loader.load("cam_fy", 0.0), loader.load("cam_cx", 0.0),
        loader.load("cam_cy", 0.0), loader.load("k1", 0.0),
        loader.load("k2", 0.0), loader.load("k3", 0.0), loader.load("k4", 0.0));
  } else if (cam_model == "PolynomialCamera") {
    cam = new vk::PolynomialCamera(
        loader.load("cam_width", 0), loader.load("cam_height", 0),
        // loader.load("scale", 1.0)
        // scale
        loader.load("cam_fx", 0.0), loader.load("cam_fy", 0.0),
        loader.load("cam_cx", 0.0), loader.load("cam_cy", 0.0),
        loader.load("cam_skew", 0.0), loader.load("k2", 0.0),
        loader.load("k3", 0.0), loader.load("k4", 0.0), loader.load("k5", 0.0),
        loader.load("k6", 0.0), loader.load("k7", 0.0));
  } else if (cam_model == "ATAN") {
    cam = new vk::ATANCamera(
        loader.load("cam_width", 0), loader.load("cam_height", 0),
        loader.load("cam_fx", 0.0), loader.load("cam_fy", 0.0),
        loader.load("cam_cx", 0.0), loader.load("cam_cy", 0.0),
        loader.load("cam_d0", 0.0));
  } else {
    FINS_LOG_ERROR("[CameraLoader] Unknown camera model: {}", cam_model);
    cam = nullptr;
    res = false;
  }
  return res;
}

inline bool loadFromFins(const std::string &ns,
                         std::vector<vk::AbstractCamera *> &cam_list) {
  fins::ParamLoader loader(ns);

  bool res = true;

  std::string global_cam_model = loader.load("cam_model", std::string(""));
  int cam_num = loader.load("cam_num", 0);

  if (cam_num <= 0) {
    FINS_LOG_WARN("[CameraLoader] cam_num is 0 or missing in {}", ns);
    return false;
  }

  for (int i = 0; i < cam_num; i++) {
    std::string cam_ns = ns + ".cam_" + std::to_string(i);
    fins::ParamLoader sub_loader(cam_ns);

    std::string cam_model = sub_loader.load("cam_model", std::string(""));

    if (cam_model == "FishPoly") {
      cam_list.push_back(new vk::PolynomialCamera(
          sub_loader.load("image_width", 0), sub_loader.load("image_height", 0),
          // sub_loader.load("scale", 1.0),
          sub_loader.load("A11", 0.0), // cam_fx
          sub_loader.load("A22", 0.0), // cam_fy
          sub_loader.load("u0", 0.0),  // cam_cx
          sub_loader.load("v0", 0.0),  // cam_cy
          sub_loader.load("A12", 0.0), // cam_skew
          sub_loader.load("k2", 0.0), sub_loader.load("k3", 0.0),
          sub_loader.load("k4", 0.0), sub_loader.load("k5", 0.0),
          sub_loader.load("k6", 0.0), sub_loader.load("k7", 0.0)));
    } else if (cam_model == "Pinhole") {
      cam_list.push_back(new vk::PinholeCamera(
          sub_loader.load("cam_width", 0), sub_loader.load("cam_height", 0),
          sub_loader.load("scale", 1.0), sub_loader.load("cam_fx", 0.0),
          sub_loader.load("cam_fy", 0.0), sub_loader.load("cam_cx", 0.0),
          sub_loader.load("cam_cy", 0.0), sub_loader.load("cam_d0", 0.0),
          sub_loader.load("cam_d1", 0.0), sub_loader.load("cam_d2", 0.0),
          sub_loader.load("cam_d3", 0.0)));
    } else {
      FINS_LOG_ERROR("[CameraLoader] Unknown sub-camera model '{}' in {}",
                     cam_model, cam_ns);
      res = false;
    }
  }

  return res;
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_