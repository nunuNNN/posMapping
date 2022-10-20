#ifndef __INTERFACE_NAVBOX_H__
#define __INTERFACE_NAVBOX_H__

#include <iostream>

typedef unsigned int UINT32;
typedef float FLOAT32;
typedef double FLOAT64;
typedef float MyVect3f[3];
typedef double MyVect3d[3];

typedef struct {
  FLOAT32 timestamp;  // units: s
  MyVect3f wm;        // units: rad
  MyVect3f vm;        // units: m/s
} imuSample;

typedef struct {
  FLOAT32 timestamp;  // units: s
  FLOAT32 odS;        // units: m
} odomSample;

typedef struct {
  FLOAT32 timestamp;  // units: s
  FLOAT64 latitude;   // units: rad
  FLOAT64 longitude;  // units: rad
  FLOAT32 altitude;   // units: m
  FLOAT32 heading;    // units: rad
  MyVect3f vel_enu;   // units: m/s
} gnssSample;

typedef struct {
  MyVect3d att, vn, pos, Patt, Pvn, Ppos;
  double t;
} StrState;

/**
 * @brief 标定参数装订
 *
 * @param lvG imu与gnss的杆臂初始值
 * @param lvO imu与odom的杆臂初始值
 */
void navbox_pos_init(MyVect3f &lvG, MyVect3f &lvO,
                     void (*PublishStateFusion)(const StrState &strState));

/**
 * @brief 将IMU数据送入系统中
 *
 * @param imu_s imu结构体
 */
void navbox_pos_sendImu(imuSample &imu_s);

/**
 * @brief 将ODOM数据送入系统中
 *
 * @param odom_s odom结构体
 */
void navbox_pos_sendOdom(odomSample &odom_s);

/**
 * @brief 将GNSS数据送入系统中
 *
 * @param gnss_s gnss结构体
 */
void navbox_pos_sendGnss(gnssSample &gnss_s);

/**
 * @brief 开始运行系统进行计算
 *
 * @param
 */
void navbox_pos_runSys();

#endif  //__INTERFACE_NAVBOX_H__
