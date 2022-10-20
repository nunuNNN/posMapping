// C/C++
#include <Python.h>
#include <math.h>
#include <omp.h>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include <thread>

// lio
#include "msgs.h"
#include "preprocess.h"
#include "slam_manager.hpp"
#include "use-ikfom.hpp"

// eigen&& pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

// ros
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

state_ikfom state_point;

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());  // 去畸变的点云
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  // 降采样的点云

double lidar_end_time = 0;

// ros相关的函数与变量
// ******************************************************************
shared_ptr<Preprocess> p_pre(new Preprocess());     // 点云预处理
shared_ptr<slamManager> p_slam(new slamManager());  // 算法处理

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;

string lid_topic, imu_topic;  // 雷达IMU的消息名
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false,
     pcd_save_en = false, path_en = true;
int pcd_save_interval = -1, pcd_index = 0;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  // printf("lidar got at: %f \r\n", msg->header.stamp.toSec());

  double timestamp_lidar = msg->header.stamp.toSec();
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  p_slam->sendLidarData(timestamp_lidar, ptr);
}

custom_messages::Imu imu_msg;
double time_diff_lidar_to_imu = 0.0;
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
  sensor_msgs::Imu::Ptr msgRos(new sensor_msgs::Imu(*msg_in));
  msgRos->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() -
                                             time_diff_lidar_to_imu);
  // printf("IMU got at: %f \r\n", msgRos->header.stamp.toSec());

  // 将ros的imu格式转化成自定义格式
  imu_msg.header.seq = msg_in->header.seq;
  imu_msg.header.stamp.sec = msgRos->header.stamp.sec;
  imu_msg.header.stamp.nsec = msgRos->header.stamp.nsec;
  imu_msg.header.frame_id = msg_in->header.frame_id;

  imu_msg.orientation.x = msg_in->orientation.x;
  imu_msg.orientation.y = msg_in->orientation.y;
  imu_msg.orientation.z = msg_in->orientation.z;
  imu_msg.orientation.w = msg_in->orientation.w;
  for (int i = 0; i < 9; ++i)
    imu_msg.orientation_covariance[i] = msg_in->orientation_covariance[i];

  imu_msg.angular_velocity.x = msg_in->angular_velocity.x;
  imu_msg.angular_velocity.y = msg_in->angular_velocity.y;
  imu_msg.angular_velocity.z = msg_in->angular_velocity.z;
  for (int i = 0; i < 9; ++i)
    imu_msg.angular_velocity_covariance[i] =
        msg_in->angular_velocity_covariance[i];

  imu_msg.linear_acceleration.x = msg_in->linear_acceleration.x;
  imu_msg.linear_acceleration.y = msg_in->linear_acceleration.y;
  imu_msg.linear_acceleration.z = msg_in->linear_acceleration.z;
  for (int i = 0; i < 9; ++i)
    imu_msg.linear_acceleration_covariance[i] =
        msg_in->linear_acceleration_covariance[i];
  ImuPtr msg(new custom_messages::Imu(imu_msg));

  p_slam->sendImuData(imu_msg);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                  state_point.offset_T_L_I) +
               state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar +
                 state_point.offset_T_L_I);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

template <typename T>
void set_posestamp(T &out) {
  out.pose.position.x = state_point.pos(0);
  out.pose.position.y = state_point.pos(1);
  out.pose.position.z = state_point.pos(2);
  out.pose.orientation.x = state_point.rot.coeffs()[0];
  out.pose.orientation.y = state_point.rot.coeffs()[1];
  out.pose.orientation.z = state_point.rot.coeffs()[2];
  out.pose.orientation.w = state_point.rot.coeffs()[3];
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped) {
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp = ros::Time().fromSec(
      lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
  set_posestamp(odomAftMapped.pose);
  pubOdomAftMapped.publish(odomAftMapped);
  // auto P = kf.get_P();
  // for (int i = 0; i < 6; i ++)
  // {
  //     int k = i < 3 ? i + 3 : i - 3;
  //     odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
  //     odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
  //     odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
  //     odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
  //     odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
  //     odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
  // }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                  odomAftMapped.pose.pose.position.y,
                                  odomAftMapped.pose.pose.position.z));
  q.setW(odomAftMapped.pose.pose.orientation.w);
  q.setX(odomAftMapped.pose.pose.orientation.x);
  q.setY(odomAftMapped.pose.pose.orientation.y);
  q.setZ(odomAftMapped.pose.pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp,
                                        "camera_init", "body"));
}

void publish_path(const ros::Publisher pubPath) {
  set_posestamp(msg_body_pose);
  msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
  msg_body_pose.header.frame_id = "camera_init";

  /*** if path is too large, the rvis will crash ***/
  static int jjj = 0;
  jjj++;
  if (jjj % 10 == 0) {
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
  }
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher &pubLaserCloudFull) {
  if (scan_pub_en) {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort
                                                       : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                          &laserCloudWorld->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull.publish(laserCloudmsg);
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      RGBpointBodyToWorld(&feats_undistort->points[i],
                          &laserCloudWorld->points[i]);
    }
    *pcl_wait_save += *laserCloudWorld;

    static int scan_wait_num = 0;
    scan_wait_num++;
    if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 &&
        scan_wait_num >= pcd_save_interval) {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") +
                            to_string(pcd_index) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      cout << "current scan saved to /PCD/" << all_points_dir << endl;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
      pcl_wait_save->clear();
      scan_wait_num = 0;
    }
  }
}

void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body) {
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                           &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body.publish(laserCloudmsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laserMapping");  // 初始化ros节点，节点名为laserMapping
  ros::NodeHandle
      nh;  // 从参数服务器读取参数值赋给变量（包括launch文件和launch读取的yaml文件中的参数）

  // ros发布相关的变量
  nh.param<bool>("publish/path_en", path_en, true);
  nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
  nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en,
                 false);  //是否保存pcd地图文件
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  // 雷达变化相关
  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
  nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
  nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
  nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
  nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);

  nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu,
                   0.0);

  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";

  /*** ROS subscribe initialization ***/
  ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
  ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
  ros::Publisher pubLaserCloudFull =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  ros::Publisher pubLaserCloudFull_body =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);

  ros::Rate rate(5000);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    bool runSys =
        p_slam->startSys(feats_undistort, feats_down_body, lidar_end_time);

    if (!runSys) continue;

    state_point = p_slam->getState();

    /******* Publish odometry *******/
    publish_odometry(pubOdomAftMapped);
    /******* Publish points *******/
    if (path_en) publish_path(pubPath);
    if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFull);
    if (scan_pub_en && scan_body_pub_en)
      publish_frame_body(pubLaserCloudFull_body);

    status = ros::ok();
    rate.sleep();
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. pcd save will largely influence the real-time performences **/
  if (pcl_wait_save->size() > 0 && pcd_save_en) {
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    cout << "current scan saved to /PCD/" << file_name << endl;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }

  return 0;
}
