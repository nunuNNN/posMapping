#include <vector>
#include <yaml-cpp/yaml.h>

// hesai driver
// #include "pandarGeneral_sdk/pandarGeneral_sdk.h"

// navbox driver
#include "data_translation.h"

// navBoxPos
#include "interface_navbox.h"

// // lio&&mapping
// #include "common_lib.h"
// #include "slam_manager.h"
// #include "use-ikfom.h"

using namespace std;

#define ROOT_DIR             std::string("../")
#define CONFIG_FILE          std::string(ROOT_DIR + std::string("config/posMapping.yaml"))


// data path
string dataPath;

// navbox driver
gnssSample gnss_sample; imuSample imu_sample; odomSample odom_sample;
u4 first_imu_time = 0;
double init_head;

// navbox pos
vector<float> leverG, leverO;


// int point_filter_num = 2, N_SCANS = 16;
// double blind = 1.0;

// shared_ptr<slamManager> p_slam(new slamManager());  // 算法处理
// state_ikfom state_point;
// PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());  // 去畸变的点云
// PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  // 降采样的点云
// double lidar_end_time = 0;

void imuCallback(const imuStr& imu_str) {
  // printf("imu time is %d \r\n", imu_str.timestamp);

  if(imu_str.dt > 1000) {
    first_imu_time = imu_str.timestamp;
    return;
  }

  imu_sample.timestamp = f4(imu_str.timestamp - first_imu_time) / 1000;
  imu_sample.vm[0] = imu_str.vm[0] * imu_str.dt / 1000;
  imu_sample.vm[1] = imu_str.vm[1] * imu_str.dt / 1000;
  imu_sample.vm[2] = imu_str.vm[2] * imu_str.dt / 1000;
  imu_sample.wm[0] = imu_str.wm[0] * imu_str.dt / 1000;
  imu_sample.wm[1] = imu_str.wm[1] * imu_str.dt / 1000;
  imu_sample.wm[2] = imu_str.wm[2] * imu_str.dt / 1000;

  navbox_pos_sendImu(imu_sample);
}

void odomCallback(const odomStr& odom_str) {
  // printf("odom time is %d \r\n", odom_str.timestamp);

  if(odom_str.dt > 1000 || 
    odom_str.timestamp < first_imu_time || 
    first_imu_time == 0) return;

  odom_sample.timestamp = f4(odom_str.timestamp - first_imu_time) / 1000;
  odom_sample.odS = odom_str.vel * odom_str.dt / 1000;

  navbox_pos_sendOdom(odom_sample);
}

void gnssCallback(const gnssStr& gnss_str) {
  // printf("gnss time is %d \r\n", gnss_str.timestamp);
  if(gnss_str.timestamp < first_imu_time || first_imu_time == 0) return;

    gnss_sample.timestamp = f4(gnss_str.timestamp - first_imu_time) / 1000;

    gnss_sample.latitude  = gnss_str.latitude;
    gnss_sample.longitude = gnss_str.longitude;
    gnss_sample.altitude  = gnss_str.altitude;

    gnss_sample.vel_enu[0] = gnss_str.vel_enu[0];
    gnss_sample.vel_enu[1] = gnss_str.vel_enu[1];
    gnss_sample.vel_enu[2] = gnss_str.vel_enu[2];
    // ENU 北偏西为正
    gnss_sample.heading = gnss_str.heading + init_head * kDEG2RAD;

    navbox_pos_sendGnss(gnss_sample);
}

// void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {

//   PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

//   int plsize = cld->points.size();
//   ptr->reserve(plsize);
//   for (int i = 0; i < plsize; i++) {
//     PointType added_pt;
//     added_pt.normal_x = 0;
//     added_pt.normal_y = 0;
//     added_pt.normal_z = 0;
//     added_pt.x = cld->points[i].x;
//     added_pt.y = cld->points[i].y;
//     added_pt.z = cld->points[i].z;
//     added_pt.intensity = cld->points[i].intensity;
//     added_pt.curvature =
//         (cld->points[i].timestamp - timestamp) * 1000;  // s to ms
//     if (i % point_filter_num == 0 && cld->points[i].ring < N_SCANS) {
//       if (added_pt.x * added_pt.x + added_pt.y * added_pt.y +
//               added_pt.z * added_pt.z >
//           blind)
//         ptr->points.push_back(added_pt);
//     }
//   }

//   p_slam->sendLidarData(timestamp, ptr);
// }


void PublishStateFusion(const StrState& strState) {
  // printf("callback time is %f \r\n", strState.t);
}

int main(int argc, char** argv) {
  /*-------------------------------------------------------------------------*/
  // read the params from .yaml and set the params
  YAML::Node config = YAML::LoadFile(CONFIG_FILE);

  dataPath  = config["data"]["path"].as<string>();
  init_head = config["navbox"]["init_head"].as<double>();
  leverG       = config["navbox"]["leverG"].as<vector<float>>();
  leverO       = config["navbox"]["leverO"].as<vector<float>>();


  /*-------------------- 初始化box激光驱动\后处理liomap -------------------------*/
  std::string navboxPath = dataPath + "232R1.dat";
  dataTranslation navBoxdata(navboxPath, imuCallback, odomCallback, gnssCallback);

  std::string hesaiPath = dataPath;
  // PandarGeneralSDK pandarGeneral(hesaiPath, lidarCallback, 0, 0, 1,
  //                                std::string("PandarXT-16"), "", "", false);
  // std::string filePath = dataPath;
  // std::ifstream fin(filePath);
  // if (fin.is_open()) {
  //   std::cout << "Open correction file " << filePath << " succeed" << std::endl;
  //   int length = 0;
  //   std::string strlidarCalibration;
  //   fin.seekg(0, std::ios::end);
  //   length = fin.tellg();
  //   fin.seekg(0, std::ios::beg);
  //   char* buffer = new char[length];
  //   fin.read(buffer, length);
  //   fin.close();
  //   strlidarCalibration = buffer;
  //   int ret = pandarGeneral.LoadLidarCorrectionFile(strlidarCalibration);
  //   if (ret != 0) {
  //     std::cout << "Load correction file from " << filePath << " failed"
  //               << std::endl;
  //   } else {
  //     std::cout << "Load correction file from " << filePath << " succeed"
  //               << std::endl;
  //   }
  // } else {
  //   std::cout << "Open correction file " << filePath << " failed" << std::endl;
  // }

  MyVect3f lvG, lvO;
  lvG[0] = leverG[0]; lvG[1] = leverG[1]; lvG[2] = leverG[2];
  lvO[0] = leverO[0]; lvO[0] = leverO[1]; lvO[0] = leverO[2];
  navbox_pos_init(lvG, lvO, PublishStateFusion);

  /*-------------------------------------------------------------------------*/
  // run dat parse && send data to sys
  navBoxdata.runDataTrans();

  /*-------------------------------------------------------------------------*/
  // run pos sys state estimation
  navbox_pos_runSys();

  /*-------------------------------------------------------------------------*/
  // pandarGeneral.Start();
  return 0;
}
