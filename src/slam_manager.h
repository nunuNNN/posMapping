
#include <math.h>
#include <fstream>
#include <mutex>
#include <thread>

#include <Eigen/Core>

#include "imu_process.h"
#include "navbox_lio.h"


class slamManager {
 public:
  slamManager();
  ~slamManager();

  void sendImuData(custom_messages::Imu &imu_msg);
  void sendLidarData(double &idar_time, PointCloudXYZI::Ptr &lidar_msg);

  bool startSys(PointCloudXYZI::Ptr &feats_undistort,
                PointCloudXYZI::Ptr &feats_down_body, double &lidar_time);

  state_ikfom getState();

 private:

  bool sync_packages(MeasureGroup &meas);

 private:
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;

  shared_ptr<ImuProcess> p_imu;  // 前端
  shared_ptr<NavBoxLio> p_lio;   // 后端

  std::mutex mtx_buffer;
  deque<double> time_buffer;
  deque<PointCloudXYZI::Ptr> lidar_buffer;
  deque<ImuConstPtr> imu_buffer;

  MeasureGroup Measures;

  double lidar_mean_scantime;
  int scan_num;
  bool lidar_pushed, flg_first_scan;

  double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;

  double lidar_end_time;
};
