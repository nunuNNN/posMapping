#include "slam_manager.h"

M3D Eye3d(M3D::Identity());
V3D Zero3d(0, 0, 0);

Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc-s.ba); 
  for(int i = 0; i < 3; i++ ){
    res(i) = s.vel[i];
    res(i + 3) =  omega[i]; 
    res(i + 12) = a_inertial[i] + s.grav[i]; 
  }
  return res;
}

Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
{
  Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
  cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
  Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  s.S2_Mx(grav_matrix, vec, 21);
  cov.template block<3, 2>(12, 21) =  grav_matrix; 
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); 
  return cov;
}

Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
  Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
  cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
  return cov;
}

slamManager::slamManager() {
  lidar_end_time = 0;
  lidar_mean_scantime = 0.0;
  scan_num = 0;

  lidar_pushed = false;
  flg_first_scan = true;

  p_imu.reset(new ImuProcess());
  p_lio.reset(new NavBoxLio());

  // 前端初始化
  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
  V3D Lidar_T_wrt_IMU(Zero3d);
  M3D Lidar_R_wrt_IMU(Eye3d);
  Lidar_T_wrt_IMU << -0.129982, -0.026843, 0.151303;
  Lidar_R_wrt_IMU << 1, 0, 0, 
                   0, 1, 0, 
                   0, 0, 1;
  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

  // 后端初始化
  double filter_size_map_min = 0.5; 
  double filter_size_surf_min = 0.5; 
  p_lio->set_filter_size(filter_size_map_min, filter_size_surf_min);
  double det_range = 100.0;
  int cube_len = 1000;
  p_lio->set_localMap_param(det_range, cube_len);
  bool need_calibr_ext = false;
  p_lio->set_calibr_ext(need_calibr_ext);

  // kf 初始化
  double epsi[23] = {0.001};
  fill(epsi, epsi + 23, 0.001);
  int NUM_MAX_ITERATIONS = 4;
  kf.init_dyn_share(get_f, df_dx, df_dw, p_lio->h_share_model,
                    NUM_MAX_ITERATIONS, epsi);
}

slamManager::~slamManager() {}

void slamManager::sendImuData(custom_messages::Imu &imu_msg) {
  double timestamp = imu_msg.header.stamp.toSec();
  mtx_buffer.lock();
  if (timestamp < last_timestamp_imu) {
    printf("imu loop back, clear buffer\r\n");
    imu_buffer.clear();
  }
  last_timestamp_imu = timestamp;

  ImuConstPtr msg(new custom_messages::Imu(imu_msg));

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
}

void slamManager::sendLidarData(double &idar_time,
                                PointCloudXYZI::Ptr &lidar_msg) {
  mtx_buffer.lock();
  if (idar_time < last_timestamp_lidar) {
    printf("lidar loop back, clear buffer\r\n");
    lidar_buffer.clear();
  }

  lidar_buffer.push_back(lidar_msg);
  time_buffer.push_back(idar_time);
  last_timestamp_lidar = idar_time;
  mtx_buffer.unlock();
}

bool slamManager::sync_packages(MeasureGroup &meas) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    if (meas.lidar->points.size() <= 1)  // time too little
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      printf("Too few input point cloud!\r\n");
    } else if (meas.lidar->points.back().curvature / double(1000) <
               0.5 * lidar_mean_scantime) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      lidar_end_time = meas.lidar_beg_time +
                       meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime +=
          (meas.lidar->points.back().curvature / double(1000) -
           lidar_mean_scantime) /
          scan_num;
    }

    meas.lidar_end_time = lidar_end_time;

    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = imu_buffer.front()->header.stamp.toSec();
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
    imu_time = imu_buffer.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time) break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

bool slamManager::startSys(PointCloudXYZI::Ptr &feats_undistort,
                           PointCloudXYZI::Ptr &feats_down_body,
                           double &lidar_time) {
  if (sync_packages(Measures)) {
    if (flg_first_scan) {
      flg_first_scan = false;
      return false;
    }
    p_imu->Process(Measures, kf, feats_undistort);
    if (feats_undistort->empty() || (feats_undistort == NULL)) {
      printf("No point, skip this scan!\r\n");
      return false;
    }

    p_lio->Process(kf, feats_undistort, feats_down_body);
    lidar_time = lidar_end_time;
    return true;
  } else {
    return false;
  }
}

state_ikfom slamManager::getState() { return kf.get_x(); }
