#include <yaml-cpp/yaml.h>
#include <deque>
#include <vector>

// hesai driver
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"

// navbox driver
#include "data_translation.h"

// navBoxPos
#include "interface_navbox.h"

// lio&&mapping
#include "common_lib.h"
#include "slam_manager.h"
#include "use-ikfom.h"

using namespace std;

#define CONFIG_FILE std::string("../config/posMapping.yaml")

/*-------------------- 相关变量的定义 --------------------*/
// data path
string dataPath;

// navbox driver
gnssSample gnss_sample;
imuSample imu_sample;
odomSample odom_sample;
u4 first_imu_time = 0;
double init_head;

// navbox pos
vector<float> leverG, leverO;
std::deque<StrState> state_msg_buffer;

// lidar driver&preprocess
int point_filter_num, scanLine;
double blind;

// LIO && Mapping
shared_ptr<slamManager> p_slam(new slamManager());  // 算法处理
state_ikfom state_point;
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());  // 去畸变的点云
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  // 降采样的点云
double lidar_end_time = 0;

/*-------------------- navbox回调函数(原始数据、导航数据) --------------------*/
void imuCallback(const imuStr& imu_str) {
  // printf("imu time is %d \r\n", imu_str.timestamp);

  if (imu_str.dt > 1000) {
    first_imu_time = imu_str.timestamp;
    return;
  }

  // 将IMU数据转换成增量，送入到pos后处理系统
  imu_sample.timestamp = f4(imu_str.timestamp - first_imu_time) / 1000;
  imu_sample.vm[0] = imu_str.vm[0] * imu_str.dt / 1000;
  imu_sample.vm[1] = imu_str.vm[1] * imu_str.dt / 1000;
  imu_sample.vm[2] = imu_str.vm[2] * imu_str.dt / 1000;
  imu_sample.wm[0] = imu_str.wm[0] * imu_str.dt / 1000;
  imu_sample.wm[1] = imu_str.wm[1] * imu_str.dt / 1000;
  imu_sample.wm[2] = imu_str.wm[2] * imu_str.dt / 1000;

  navbox_pos_sendImu(imu_sample);

  // 将IMU数据发送到LIO&Mapping系统
  custom_messages::Imu imu_msg;
  imu_msg.header.stamp.sec = (imu_str.timestamp - first_imu_time) / 1000;
  imu_msg.header.stamp.nsec =
      ((imu_str.timestamp - first_imu_time) % 1000) * 1e6;

  imu_msg.angular_velocity.x = imu_str.wm[0];
  imu_msg.angular_velocity.y = imu_str.wm[1];
  imu_msg.angular_velocity.z = imu_str.wm[2];
  imu_msg.linear_acceleration.x = imu_str.vm[0];
  imu_msg.linear_acceleration.y = imu_str.vm[1];
  imu_msg.linear_acceleration.z = imu_str.vm[2];

  ImuPtr msg(new custom_messages::Imu(imu_msg));
  p_slam->sendImuData(imu_msg);
}

void odomCallback(const odomStr& odom_str) {
  // printf("odom time is %d \r\n", odom_str.timestamp);

  if (odom_str.dt > 1000 || odom_str.timestamp < first_imu_time ||
      first_imu_time == 0)
    return;

  odom_sample.timestamp = f4(odom_str.timestamp - first_imu_time) / 1000;
  odom_sample.odS = odom_str.vel * odom_str.dt / 1000;

  navbox_pos_sendOdom(odom_sample);
}

void gnssCallback(const gnssStr& gnss_str) {
  // printf("gnss time is %d \r\n", gnss_str.timestamp);
  if (gnss_str.timestamp < first_imu_time || first_imu_time == 0) return;

  gnss_sample.timestamp = f4(gnss_str.timestamp - first_imu_time) / 1000;

  gnss_sample.latitude = gnss_str.latitude;
  gnss_sample.longitude = gnss_str.longitude;
  gnss_sample.altitude = gnss_str.altitude;

  gnss_sample.vel_enu[0] = gnss_str.vel_enu[0];
  gnss_sample.vel_enu[1] = gnss_str.vel_enu[1];
  gnss_sample.vel_enu[2] = gnss_str.vel_enu[2];
  // ENU 北偏西为正
  gnss_sample.heading = gnss_str.heading + init_head * kDEG2RAD;

  navbox_pos_sendGnss(gnss_sample);
}

void PublishStateFusion(const StrState& strState) {
  // printf("callback time is %f; pos is %f %f %f \r\n", strState.t,
  //        strState.pos[0], strState.pos[1], strState.pos[2]);
  state_msg_buffer.push_front(strState);
}

/*-------------------- 激光雷达回调函数(畸变激光雷达数据) --------------------*/
void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

  int plsize = cld->points.size();
  ptr->reserve(plsize);
  for (int i = 0; i < plsize; i++) {
    PointType added_pt;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = cld->points[i].x;
    added_pt.y = cld->points[i].y;
    added_pt.z = cld->points[i].z;
    added_pt.intensity = cld->points[i].intensity;
    added_pt.curvature =
        (cld->points[i].timestamp - timestamp) * 1000;  // s to ms
    if (i % point_filter_num == 0 && cld->points[i].ring < scanLine) {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y +
              added_pt.z * added_pt.z >
          blind)
        ptr->points.push_back(added_pt);
    }
  }
  p_slam->sendLidarData(timestamp, ptr);
}

/*-------------------- 根据激光雷达的时间查找对用的位姿 --------------------*/

int main(int argc, char** argv) {
  /*-------------------  通过yaml加载参数 -------------------*/
  // read the params from .yaml and set the params
  YAML::Node config = YAML::LoadFile(CONFIG_FILE);

  dataPath = config["data"]["path"].as<string>();

  init_head = config["navbox"]["init_head"].as<double>();
  leverG = config["navbox"]["leverG"].as<vector<float>>();
  leverO = config["navbox"]["leverO"].as<vector<float>>();

  scanLine = config["preprocess"]["scan_line"].as<int>();
  blind = config["preprocess"]["blind"].as<double>();
  point_filter_num = config["preprocess"]["filter_num"].as<int>();

  /*-------------------- 初始化box驱动 --------------------*/
  std::string navboxPath = dataPath + "232R1.dat";
  dataTranslation navBoxdata(navboxPath, imuCallback, odomCallback,
                             gnssCallback);

  /*-------------------- 初始化禾赛激光驱动 --------------------*/
  // std::string hesaiPath = dataPath;
  // PandarGeneralSDK pandarGeneral(hesaiPath, lidarCallback, 0, 0, 1,
  //                                std::string("PandarXT-16"), "", "", false);
  // std::string filePath = dataPath;
  // std::ifstream fin(filePath);
  // if (fin.is_open()) {
  //   std::cout << "Open correction file " << filePath << " succeed" <<
  //   std::endl; int length = 0; std::string strlidarCalibration; fin.seekg(0,
  //   std::ios::end); length = fin.tellg(); fin.seekg(0, std::ios::beg); char*
  //   buffer = new char[length]; fin.read(buffer, length); fin.close();
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
  //   std::cout << "Open correction file " << filePath << " failed" <<
  //   std::endl;
  // }

  /*-------------------- 播放dat文件,数据进回调函数 --------------------*/
  // run dat parse && send data to sys
  navBoxdata.runDataTrans();

  /*-------------------- 禾赛激光雷达开始播放 --------------------*/
  // pandarGeneral.Start();

  /*-------------------- 初始化pos后处理 --------------------*/
  MyVect3f lvG, lvO;
  lvG[0] = leverG[0];
  lvG[1] = leverG[1];
  lvG[2] = leverG[2];
  lvO[0] = leverO[0];
  lvO[0] = leverO[1];
  lvO[0] = leverO[2];
  navbox_pos_init(lvG, lvO, PublishStateFusion);

  /*-------------------- 对数据进行后处理估计 --------------------*/
  // run pos sys state estimation
  navbox_pos_runSys();

  /*-------------------- 进行LIO&mapping --------------------*/
  while (true) {
    StrState tmp = state_msg_buffer.front();
    state_msg_buffer.pop_front();
    printf("callback time is %f \r\n", tmp.t);

    if(state_msg_buffer.size() == 0) break;
    // bool runSys =
    //     p_slam->startSys(feats_undistort, feats_down_body, lidar_end_time);

    // if (!runSys) continue;

    // state_point = p_slam->getState();
  }

  return 0;
}
