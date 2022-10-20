#include "data_translation.h"

T_User_FrameHead s_Head[] = {
    {{0xAA, 0x44, 0x12, 0x1C}, rcvGnssHead},
    {{0x7E, 0xE7, 0xAA, 0x15}, rcvOdom},
};

#define _switchState(a, b) \
  pMgr->state = a;         \
  pMgr->ms_count = b

void (*imuCallback)(const imuStr& imu_str);
void (*odomCallback)(const odomStr& odom_str);
void (*gnssCallback)(const gnssStr& gnss_str);

dataTranslation::dataTranslation(const std::string &dat_path,
                      void (*iCb)(const imuStr& imu_str),
                      void (*oCb)(const odomStr& odom_str),
                      void (*gCb)(const gnssStr& gnss_str))
{
  datPath = dat_path;

  imuCallback = iCb; odomCallback = oCb; gnssCallback = gCb;

  g_timeStatus = 0; g_GpsweekNum = 0; g_GpsMsCnt = 0; g_running_ms = 0;

  _gnss_pos_time_ms_psd = 0;
  _gnss_vel_time_ms_psd = 0;
  _gnss_xyz_time_ms_psd = 0;
  _gnss_yaw_time_ms_psd = 0;
  _imu_time_ms_psd = 0;
  _odom_time_ms_psd = 0;

  s_GNSSfrmFinishedFlg = 0;
  imu_is_update = false;
  odo_is_update = false;
}

void dataTranslation::resetCollectBuf() {
  memset(&s_RcvMgr, 0, sizeof(T_User_RcvMgr));
}

void dataTranslation::parserRcvCmd(u1 cb) {
  T_User_RcvMgr* pMgr = &s_RcvMgr;
  u1* pBuff = pMgr->Buffer;
  i4 i;
  u2 msgIdx;

  pBuff[pMgr->idx++] = cb;

  if (pMgr->idx >= RECV_BUFFER_SIZE) {
    resetCollectBuf();
    _switchState(idle, g_running_ms);
    pMgr->idx = 0;
  }

  switch (pMgr->state) {
    case idle:
      if (pMgr->idx >= 4) {
        for (i = 0; i < (sizeof(s_Head) / sizeof(T_User_FrameHead)); i++) {
          if (memcmp(&pBuff[pMgr->idx - 4], &s_Head[0].head[0], 4) == 0) {
            pMgr->idx = 4;
            pMgr->rcvHeadLenth = 28;
            _switchState(s_Head[0].state, g_running_ms);
          } else if (memcmp(&pBuff[pMgr->idx - 4], &s_Head[1].head[0], 4) ==
                     0) {
            pMgr->idx = 4;
            pMgr->rcvLen = 18;
            _switchState(s_Head[1].state, g_running_ms);
          }
        }
      }
      break;

    case rcvGnssHead:
      if (pMgr->idx == pMgr->rcvHeadLenth) {
        parseRcvFrmHead(pBuff);
        memcpy(&msgIdx, &pBuff[offset_msg_id], sizeof(u2));
        switch (msgIdx) {
          case msg_pos_id:
            pMgr->rcvLen = 104;
            _switchState(rcvPos, g_running_ms);
            break;
          case msg_vel_id:
            pMgr->rcvLen = 76;
            _switchState(rcvVel, g_running_ms);
            break;
          case msg_xyz_id:
            pMgr->rcvLen = 144;
            _switchState(rcvXYZ, g_running_ms);
            break;
          case msg_heading_id:
            pMgr->rcvLen = 76;
            _switchState(rcvHeading, g_running_ms);
            break;
          case msg_imu_id:
            pMgr->rcvLen = 72;
            _switchState(rcvImu, g_running_ms);
            break;
          default:
            break;
        }
      }
      break;
    case rcvPos:
      if (pMgr->idx == pMgr->rcvLen) {
        RcvPosFrame(pBuff, pMgr->rcvLen);
        resetCollectBuf();
        _switchState(idle, g_running_ms);
        pMgr->idx = 0;
      }
      break;
    case rcvVel:
      if (pMgr->idx == pMgr->rcvLen) {
        RcvVelFrame(pBuff, pMgr->rcvLen);
        resetCollectBuf();
        _switchState(idle, g_running_ms);
        pMgr->idx = 0;
      }
      break;
    case rcvXYZ:
      if (pMgr->idx == pMgr->rcvLen) {
        RcvXYZFrame(pBuff, pMgr->rcvLen);
        resetCollectBuf();
        _switchState(idle, g_running_ms);
        pMgr->idx = 0;
      }
      break;
    case rcvHeading:
      if (pMgr->idx == pMgr->rcvLen) {
        RcvHeadingFrame(pBuff, pMgr->rcvLen);
        resetCollectBuf();
        _switchState(idle, g_running_ms);
        pMgr->idx = 0;
      }
      break;
    case rcvImu:
      if (pMgr->idx == pMgr->rcvLen) {
        RcvImuFrame(pBuff, pMgr->rcvLen);
        resetCollectBuf();
        _switchState(idle, g_running_ms);
        pMgr->idx = 0;
      }
      break;
    case rcvOdom:
      if (pMgr->idx == pMgr->rcvLen) {
        RcvOdomFrame(pBuff, pMgr->rcvLen);
        resetCollectBuf();
        _switchState(idle, g_running_ms);
        pMgr->idx = 0;
      }
      break;
    default:
      resetCollectBuf();
      _switchState(idle, g_running_ms);
      pMgr->idx = 0;
      break;
  }
}

bool dataTranslation::parseRcvFrmHead(u1* pBuff) {
  u2 msgIdx;
  g_timeStatus = pBuff[offset_time_status];
  if (g_timeStatus <= 20) {
    return false;
  }

  memcpy(&msgIdx, &pBuff[offset_msg_id], sizeof(u2));
  if (msgIdx == msg_pos_id) {
    memcpy(&g_GpsweekNum, &pBuff[offset_week], sizeof(u2));
    memcpy(&g_GpsMsCnt, &pBuff[offset_msCnt], sizeof(u4));
  }

  return true;
}

void dataTranslation::RcvPosFrame(u1* pBuff, u2 len) {
  memset(posFrmBuff, 0, sizeof(u1) * 110);
  memcpy(posFrmBuff, pBuff, sizeof(u1) * len);

  // parse bestpos msg
  RawHeader header;

  memcpy(&header, posFrmBuff, 28);         // parse header
  memcpy(&bestposa, posFrmBuff + 28, 72);  // parse gnss position

  bestposa.GpsTime = header.GpsTime;
  bestposa.Week = header.Week;
  bestposa.GpsTime_pst = _gnss_pos_time_ms_psd;

  // if(bestposa.pos_type == NARROW_INT && bestposa.solnSVs > 10 &&
  // bestposa.sol_status == SOL_COMPUTED)
  { s_GNSSfrmFinishedFlg |= 0x1; }

  _gnss_pos_time_ms_psd = bestposa.GpsTime;
  return;
}

void dataTranslation::RcvVelFrame(u1* pBuff, u2 len) {
  memset(velFrmBuff, 0, sizeof(u1) * 80);
  memcpy(velFrmBuff, pBuff, sizeof(u1) * len);

  // parse bestvela msg
  RawHeader header;
  memcpy(&header, velFrmBuff, 28);
  memcpy(&bestvela, velFrmBuff + 28, 44);

  bestvela.GpsTime = header.GpsTime;
  bestvela.Week = header.Week;
  bestvela.GpsTime_pst = _gnss_vel_time_ms_psd;
  _gnss_vel_time_ms_psd = bestvela.GpsTime;

  s_GNSSfrmFinishedFlg |= 0x2;
  return;
}

void dataTranslation::RcvXYZFrame(u1* pBuff, u2 len) {
  memset(xyzFrmBuff, 0, sizeof(u1) * 144);
  memcpy(xyzFrmBuff, pBuff, sizeof(u1) * len);

  // parse bestxyza msg
  RawHeader header;
  memcpy(&header, xyzFrmBuff, 28);
  memcpy(&bestxyza, xyzFrmBuff + 28, 112);

  bestxyza.GpsTime = header.GpsTime;
  bestxyza.Week = header.Week;
  bestxyza.GpsTime_pst = _gnss_xyz_time_ms_psd;

  // if(bestxyza.pos_type == NARROW_INT && bestxyza.vel_type == DOPPLER_VELOCITY
  // &&
  //     bestxyza.pos_sol_status == SOL_COMPUTED && bestxyza.vel_sol_status ==
  //     SOL_COMPUTED)
  { s_GNSSfrmFinishedFlg |= 0x8; }
  _gnss_xyz_time_ms_psd = bestxyza.GpsTime;

  return;
}

void dataTranslation::RcvHeadingFrame(u1* pBuff, u2 len) {
  memset(headingFrmBuff, 0, sizeof(u1) * 80);
  memcpy(headingFrmBuff, pBuff, sizeof(u1) * len);

  // parse headinga msg

  RawHeader header;

  memcpy(&header, headingFrmBuff, 28);
  memcpy(&headinga, headingFrmBuff + 28, 44);

  headinga.GpsTime = header.GpsTime;
  headinga.Week = header.Week;
  headinga.GpsTime_pst = _gnss_yaw_time_ms_psd;

  // if ( fabsf(headinga.base_length - base_line) < 0.05 &&
  //      headinga.pos_type == NARROW_INT &&
  // 	 headinga.sol_status == SOL_COMPUTED)
  { s_GNSSfrmFinishedFlg |= 0x4; }

  _gnss_yaw_time_ms_psd = headinga.GpsTime;

  return;
}

void dataTranslation::RcvImuFrame(u1* pBuff, u2 len) {
  memset(imuFrmBuff, 0, sizeof(u1) * 80);
  memcpy(imuFrmBuff, pBuff, sizeof(u1) * len);

  // parse imu msg
  RawHeader header;

  memcpy(&header, imuFrmBuff, 28);
  memcpy(&rawImu, imuFrmBuff + 28, 40);

  rawImu.GpsTime = header.GpsTime;
  rawImu.Week = header.Week;
  rawImu.GpsTime_pst = _imu_time_ms_psd;

  if (abs(rawImu.acce_x + rawImu.acce_x + rawImu.acce_x) > 1000)  // 判断非法值
  {
    rawImu.acce_x = lastRawImu.acce_x;
    rawImu.acce_y = lastRawImu.acce_y;
    rawImu.acce_z = lastRawImu.acce_z;
  }
  if (abs(rawImu.gyro_x + rawImu.gyro_y + rawImu.gyro_z) > 4000)  // 判断非法值
  {
    rawImu.gyro_x = lastRawImu.gyro_x;
    rawImu.gyro_y = lastRawImu.gyro_y;
    rawImu.gyro_z = lastRawImu.gyro_z;
  }

  imu_is_update = true;
  _imu_time_ms_psd = rawImu.GpsTime;

  lastRawImu = rawImu;
  return;
}

void dataTranslation::RcvOdomFrame(u1* pBuff, u2 len) {
  memset(odomFrmBuff, 0, sizeof(u1) * 18);
  memcpy(odomFrmBuff, pBuff, sizeof(u1) * len);

  // parse imu msg
  OdomHeader header;

  memcpy(&header, odomFrmBuff, 4);
  memcpy(&rawOdom, odomFrmBuff + 4, 16);
  rawOdom.GpsTime_pst = _odom_time_ms_psd;

  odo_is_update = true;
  _odom_time_ms_psd = rawOdom.GpsTime;

  return;
}

void dataTranslation::runDataTrans()
{
  // load data file 
  FILE* fp = fopen(datPath.c_str(), "rb");
  if ((fp) == NULL) {
    perror("fail to read");
    exit(1);
  }

  // define data struct
  imuStr imu_str;
  odomStr odom_str;
  gnssStr gnss_str;

  f4 coeff = 0.85; f4 last_vel = 0.0;

  u1 ch;
  while (!feof(fp)) {
    ch = fgetc(fp);

    parserRcvCmd(ch);

    if (imu_is_update) {
      imu_is_update = false;
      // printf("cur imu time is %d \r\n", rawImu.GpsTime);

      imu_str.dt = rawImu.GpsTime - rawImu.GpsTime_pst;
      imu_str.timestamp = rawImu.GpsTime;
      imu_str.vm[0] = rawImu.acce_x * 0.004 * kGRAVITY;
      imu_str.vm[1] = rawImu.acce_y * 0.004 * kGRAVITY;
      imu_str.vm[2] = rawImu.acce_z * 0.004 * kGRAVITY;
      imu_str.wm[0] = (rawImu.gyro_x * 0.0153) * kDEG2RAD;
      imu_str.wm[1] = (rawImu.gyro_y * 0.0153) * kDEG2RAD;
      imu_str.wm[2] = (rawImu.gyro_z * 0.0153) * kDEG2RAD;

      imuCallback(imu_str);
    }

    if (odo_is_update) {
      odo_is_update = false;
      // printf("cur odo time is %d \r\n", rawOdom.GpsTime);

      //////////////////////////////////////////////////////
      i4 curr_pulse = rawOdom.left_speed;
      if (rawOdom.right_speed == 1310720) curr_pulse = 0;
      if (rawOdom.right_speed == 1310721) curr_pulse *= -1;

      // (2*PI*r)*(num/1000)/(d_time/1000);
      f4 curr_vel = 2 * 3.1415926 * 0.335 * curr_pulse /
                    (rawOdom.GpsTime - rawOdom.GpsTime_pst);
      f4 curr_vel_filer = coeff * last_vel + (1.0 - coeff) * curr_vel;
      last_vel = curr_vel_filer;
      //////////////////////////////////////////////////////

      odom_str.dt = rawOdom.GpsTime - rawOdom.GpsTime_pst;
      odom_str.timestamp = rawOdom.GpsTime;
      odom_str.vel = curr_vel_filer;

      odomCallback(odom_str);
    }

    if ((s_GNSSfrmFinishedFlg & 0x0f) == 0x0f) {
      s_GNSSfrmFinishedFlg = 0;
      // printf("cur gnss time is %d %d %d %d\r\n", bestposa.GpsTime,
      // bestxyza.GpsTime, headinga.GpsTime, bestvela.GpsTime);

      if ((bestposa.GpsTime - bestxyza.GpsTime) != 0 ||
          (bestposa.GpsTime - headinga.GpsTime) != 0) {
        printf("gnss timestamp not alignment !!! \r\n");
      }

      gnss_str.timestamp = bestposa.GpsTime;

      gnss_str.latitude = bestposa.latitude * kDEG2RAD;
      gnss_str.longitude = bestposa.longitude * kDEG2RAD;
      gnss_str.altitude = bestposa.altitude;

      double cosB = cos(gnss_str.latitude);
      double cosL = cos(gnss_str.longitude);
      double sinB = sin(gnss_str.latitude);
      double sinL = sin(gnss_str.longitude);

      float Alpha =
          1 + (bestposa.altitude /
               sqrt(fabs(ae * ae * cosB * cosB + ap * ap * sinB * sinB)));

      double Ve = (1 / Alpha) * (-sinL * bestxyza.v_x + cosL * bestxyza.v_y);
      double Vn =
          (1 / Alpha) * (-cosL * sinB * bestxyza.v_x -
                         sinL * sinB * bestxyza.v_y + cosB * bestxyza.v_z);
      double Vu = cosL * cosB * bestxyza.v_x + sinL * cosB * bestxyza.v_y +
                  sinB * bestxyza.v_z;

      gnss_str.vel_enu[0] = Ve;
      gnss_str.vel_enu[1] = Vn;
      gnss_str.vel_enu[2] = Vu;

      // ENU 北偏西为正
      gnss_str.heading = (headinga.heading) * kDEG2RAD;

      gnssCallback(gnss_str);
    }
  }

  fclose(fp);
}
