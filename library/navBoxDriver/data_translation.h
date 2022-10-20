#ifndef __DATA_TRANSLATION_H
#define __DATA_TRANSLATION_H

#include<string>
#include<cstring>
#include<cmath>

#include "data_structure.h"
#include "parse.h"

typedef struct {
  u4 timestamp;  // units: ms
  u4 dt;         // units: ms
  MyVect3f wm;   // units: rad/s
  MyVect3f vm;   // units: m/s*s
} imuStr;

typedef struct {
  u4 timestamp;  // units: ms
  u4 dt;         // units: ms
  f4 vel;        // units: m/s
} odomStr;

typedef struct {
  u4 timestamp;  // units: ms
  f8 latitude;   // units: rad
  f8 longitude;  // units: rad
  f4 altitude;   // units: m
  f4 heading;    // units: rad
  MyVect3f vel_enu;   // units: m/s
} gnssStr;


class dataTranslation {
 public:
  dataTranslation(const std::string &dat_path,
              void (*iCb)(const imuStr& imu_str),
              void (*oCb)(const odomStr& odom_str),
              void (*gCb)(const gnssStr& gnss_str));
  ~dataTranslation() {};

  void runDataTrans();

 private:
  void resetCollectBuf();
  void parserRcvCmd(u1 cb);
  // GNSS parse
  bool parseRcvFrmHead(u1* pBuff);
  void RcvPosFrame(u1* pBuff, u2 len);
  void RcvVelFrame(u1* pBuff, u2 len);
  void RcvXYZFrame(u1* pBuff, u2 len);
  void RcvHeadingFrame(u1* pBuff, u2 len);
  // IMU&&ODOM parse
  void RcvImuFrame(u1* pBuff, u2 len);
  void RcvOdomFrame(u1* pBuff, u2 len);

 private:
  std::string datPath;
  T_User_RcvMgr s_RcvMgr;

  u1 g_timeStatus;
  u2 g_GpsweekNum;
  u4 g_GpsMsCnt;
  u4 g_running_ms;

  u4 _gnss_pos_time_ms_psd;
  u4 _gnss_vel_time_ms_psd;
  u4 _gnss_xyz_time_ms_psd;
  u4 _gnss_yaw_time_ms_psd;
  u4 _imu_time_ms_psd;
  u4 _odom_time_ms_psd;

  u1 posFrmBuff[110];
  u1 velFrmBuff[80];
  u1 xyzFrmBuff[144];
  u1 headingFrmBuff[80];
  u1 imuFrmBuff[80];
  u1 odomFrmBuff[18];

  u1 s_GNSSfrmFinishedFlg;
  bool imu_is_update;
  bool odo_is_update;

  RawGnssPos bestposa;
  RawGnssVel bestvela;
  RawGnssXYZ bestxyza;
  RawGnssYaw headinga;
  RawImuOri rawImu;
  RawImuOri lastRawImu;
  RawOdomVel rawOdom;
};

#endif  //__DATA_TRANSLATION_H
