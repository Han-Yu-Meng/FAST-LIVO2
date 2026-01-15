/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"

LIVMapper::LIVMapper(fins::Node* fins_node_)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
  fins_node = fins_node_;

  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters();
  VoxelMapConfig voxel_config;
  loadVoxelConfig(voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;

  initializeComponents();
  path.header.stamp = get_wall_time();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters()
{
  fins::ParamLoader common("FastLIVO.common");

  ros_driver_fix_en = common.get("ros_driver_bug_fix", false);
  img_en = common.get("img_en", 1);
  lidar_en = common.get("lidar_en", 1);

  fins::ParamLoader vio("FastLIVO.vio");

  normal_en = vio.get("normal_en", true);
  inverse_composition_en = vio.get("inverse_composition_en", false);
  max_iterations = vio.get("max_iterations", 5);
  IMG_POINT_COV = vio.get("img_point_cov", 100.0);
  raycast_en = vio.get("raycast_en", false);
  exposure_estimate_en = vio.get("exposure_estimate_en", true);
  inv_expo_cov = vio.get("inv_expo_cov", 0.2);
  grid_size = vio.get("grid_size", 5);
  patch_size = vio.get("patch_size", 8);
  grid_n_height = vio.get("grid_n_height", 17);
  patch_pyrimid_level = vio.get("patch_pyrimid_level", 3);
  outlier_threshold = vio.get("outlier_threshold", 1000.0);

  fins::ParamLoader time_offset("FastLIVO.time_offset");
  exposure_time_init = time_offset.get("exposure_time_init", 0.0);
  img_time_offset = time_offset.get("img_time_offset", 0.0);
  imu_time_offset = time_offset.get("imu_time_offset", 0.0);
  lidar_time_offset = time_offset.get("lidar_time_offset", 0.0);
  imu_prop_enable = time_offset.get("imu_rate_odom", false);
  gravity_align_en = time_offset.get("gravity_align_en", false);
  
  fins::ParamLoader evo("FastLIVO.evo");
  seq_name = evo.get("seq_name", "01");
  pose_output_en = evo.get("pose_output_en", false);

  fins::ParamLoader imu("FastLIVO.imu");
  gyr_cov = imu.get("gyr_cov", 1.0);
  acc_cov = imu.get("acc_cov", 1.0);
  imu_int_frame = imu.get("imu_int_frame", 3);
  imu_en = imu.get("imu_en", false);
  gravity_est_en = true;
  ba_bg_est_en = true;

  fins::ParamLoader pre("FastLIVO.preprocess");
  p_pre->blind = pre.get("blind", 0.01);
  filter_size_surf_min = pre.get("filter_size_surf", 0.5);
  hilti_en = pre.get("hilti_en", false);
  p_pre->lidar_type = static_cast<LID_TYPE>(pre.get<int>("lidar_type", static_cast<int>(AVIA)));
  p_pre->N_SCANS = pre.get("scan_line", 6);
  p_pre->point_filter_num = pre.get("point_filter_num", 3);
  p_pre->feature_enabled = pre.get("feature_extract_enabled", false);

  fins::ParamLoader extrin("FastLIVO.extrin_calib");
  extrinT = extrin.get("extrinsic_T", vector<double>{0.0, 0.0, 0.0});
  extrinR = extrin.get("extrinsic_R", vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  cameraextrinT = extrin.get("Pcl", vector<double>{0.0, 0.0, 0.0});
  cameraextrinR = extrin.get("Rcl", vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  fins::ParamLoader publish("FastLIVO.publish");
  blind_rgb_points = publish.get("blind_rgb_points", 0.01);
  pub_scan_num = publish.get("pub_scan_num", 1);
  pub_effect_point_en = publish.get("pub_effect_point_en", false);
  dense_map_en = publish.get("dense_map_en", false);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents() 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromFins("FastLIVO.camera", vio_manager->cam)) 
    throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    fins_node->logger->info("Gravity Alignment Starts");
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    fins_node->logger->info("Gravity Alignment Finished");
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    fins_node->logger->warn("[ VIO ] No point!!!");
    return;
  }

  fins_node->logger->info("[ VIO ] Raw feature num: {}", pcl_w_wait_pub->points.size());

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(vio_manager);
}

void LIVMapper::handleLIO() 
{  
  euler_cur = RotMtoEuler(_state.rot_end);

  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();
  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
    
  if (!lidar_map_inited) {
    std::lock_guard<std::mutex> lock(map_init_mutex);
    
    // Double-check after acquiring lock
    if (!lidar_map_inited) {
      try {
        voxelmap_manager->BuildVoxelMap();
        
        lidar_map_inited = true;
      } catch (const std::exception& e) {
        fins_node->logger->error("[handleLIO] ❌ Exception in BuildVoxelMap(): {}", e.what());
        throw;
      } catch (...) {
        fins_node->logger->error("[handleLIO] ❌ Unknown exception in BuildVoxelMap()");
        throw;
      }
    }
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  euler_cur = RotMtoEuler(_state.rot_end);

  Eigen::Quaterniond q_eigen = Eigen::AngleAxisd(euler_cur(2), Eigen::Vector3d::UnitZ()) * 
                              Eigen::AngleAxisd(euler_cur(1), Eigen::Vector3d::UnitY()) * 
                              Eigen::AngleAxisd(euler_cur(0), Eigen::Vector3d::UnitX());

  geoQuat.w = q_eigen.w();
  geoQuat.x = q_eigen.x();
  geoQuat.y = q_eigen.y();
  geoQuat.z = q_eigen.z();

  publish_odometry();

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  fins_node->logger->info("[ LIO ] Update Voxel Map with {} points", voxelmap_manager->pv_list_.size());
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(vio_manager);
  publish_path();
  publish_tf();

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
}

void LIVMapper::trigger() 
{
  static int trigger_count = 0;
  trigger_count++;
  
  // fins_node->logger->info("[trigger] ========== CALL #{} ==========", trigger_count);
  
  if (sync_packages(LidarMeasures)) 
  {
    fins_node->logger->info("[trigger] sync_packages returned TRUE, processing data...");
    fins_node->logger->info("[trigger] LidarMeasures.lio_vio_flg: {}", (int)LidarMeasures.lio_vio_flg);
    
    handleFirstFrame();
    fins_node->logger->info("[trigger] handleFirstFrame() completed");
    
    processImu();
    fins_node->logger->info("[trigger] processImu() completed");
    
    stateEstimationAndMapping();
    fins_node->logger->info("[trigger] stateEstimationAndMapping() completed");
  }
  else
  {
    // fins_node->logger->debug("[trigger] sync_packages returned FALSE, no data to process");
  }
  
  // fins_node->logger->info("[trigger] ========== END #{} ==========", trigger_count);
}

void LIVMapper::push_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  imu_cbk(msg); 
}

void LIVMapper::push_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  standard_pcl_cbk(msg);
}

void LIVMapper::push_livox(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg) {
  livox_pcl_cbk(msg);
}

void LIVMapper::push_img(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  img_cbk(msg);
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}


void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::Ptr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  if (abs(last_timestamp_imu - toSec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - toSec(msg->header.stamp);
    fins_node->logger->warn("Self sync IMU and LiDAR, HARD time lag is {} ", timediff_imu_wrt_lidar - 0.100);
  }

  double cur_head_time = toSec(msg->header.stamp);
  if (cur_head_time < last_timestamp_lidar)
  {
    lid_raw_data_buffer.clear();
  }
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = toRclTime(toSec(msg->header.stamp) - imu_time_offset);
  double timestamp = toSec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    fins_node->logger->warn("IMU and LiDAR not synced! delta time: {} .", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = toRclTime(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    fins_node->logger->error("IMU loop back, offset: {}", last_timestamp_imu - timestamp);
    return;
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::msg::Image::Ptr msg(new sensor_msgs::msg::Image(*msg_in));

  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  double msg_header_time = toSec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  fins_node->logger->info("Get image, its header time: {:.6f}", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    fins_node->logger->error("Image loop back, offset: {}", last_timestamp_img - msg_header_time);
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    fins_node->logger->warn("Image need Jumps: {:.6f}", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  last_timestamp_img = img_time_correct;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      meas.lidar = lid_raw_data_buffer.front();
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    {
      return false;
    }

    struct MeasureGroup m;

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (toSec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO;
    meas.measures.push_back(m);
    lidar_pushed = false;
    return true;

    break;
  }

  case LIVO:
  {
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    switch (last_lio_vio_flg)
    {
    case WAIT:
    case VIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = toSec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        fins_node->logger->error("[ Data Cut ] Throw one image frame!");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        return false;
      }

      struct MeasureGroup m;

      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (toSec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (toSec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      meas.measures.clear();
      double imu_time = toSec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false;
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    fins_node->logger->error("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  fins_node->logger->error("Out of sync_packages function unexpectedly.");
}


cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::publish_frame_world(VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;

  auto data_time = get_ros_time(LidarMeasures.last_lio_update_time);

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  if (slam_mode_ == LIVO && vio_manager->new_frame_ != nullptr) {
    PointCloudXYZRGB::Ptr temp_rgb_cloud(new PointCloudXYZRGB());
    cv::Mat img_rgb = vio_manager->img_rgb;
    
    for (const auto& pt : pcl_w_wait_pub->points) {
      Eigen::Vector3d p_w(pt.x, pt.y, pt.z);
      Eigen::Vector2d pc = vio_manager->new_frame_->w2c(p_w);
      
      if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) {
        PointTypeRGB p;
        p.x = pt.x; p.y = pt.y; p.z = pt.z;
        
        Eigen::Vector3f pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
        p.r = pixel[2]; // BGR -> R
        p.g = pixel[1]; // BGR -> G
        p.b = pixel[0]; // BGR -> B
        temp_rgb_cloud->push_back(p);
      }
    }
    
    pcl::toROSMsg(*temp_rgb_cloud, laserCloudmsg);

    laserCloudmsg.header.stamp = data_time; 
    laserCloudmsg.header.frame_id = "camera_init"; 
    
    fins_node->send<0>(laserCloudmsg, fins::now());
  } 
  // else {
  //   pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
  // }

  if (LidarMeasures.lio_vio_flg == VIO || slam_mode_ != LIVO) {
    PointCloudXYZI().swap(*pcl_w_wait_pub);
  }
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_tf()
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = get_ros_time(LidarMeasures.last_lio_update_time);
  transform.header.frame_id = "camera_init";
  transform.child_frame_id = "livox_frame";

  transform.transform.translation.x = _state.pos_end(0);
  transform.transform.translation.y = _state.pos_end(1);
  transform.transform.translation.z = _state.pos_end(2);
  transform.transform.rotation = geoQuat;

  fins_node->send<3>(transform, fins::now());
}

void LIVMapper::publish_odometry()
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "livox_frame";
  odomAftMapped.header.stamp = get_ros_time(LidarMeasures.last_lio_update_time);
  set_posestamp(odomAftMapped.pose.pose);

  fins_node->send<2>(odomAftMapped, fins::now());
}

// void LIVMapper::publish_mavros()
// {
//   msg_body_pose.header.stamp = ros::Time::now();
//   msg_body_pose.header.frame_id = "camera_init";
//   set_posestamp(msg_body_pose.pose);
//   fins_node->send(msg_body_pose, fins::now());
// }

void LIVMapper::publish_path()
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = get_wall_time();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  fins_node->send<1>(path, fins::now());
}