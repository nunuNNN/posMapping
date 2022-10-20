#ifndef __DATA_STRUCTURE_H
#define __DATA_STRUCTURE_H

typedef signed char i1;
typedef unsigned char u1;
typedef short i2;
typedef unsigned short u2;
typedef int i4;
typedef unsigned int u4;
typedef float f4;
typedef double f8;
typedef float MyVect3f[3];
typedef double myVect3d[3];


#define RECV_BUFFER_SIZE (3072)

enum {
  idle = 0,
  rcvGnssHead,
  rcvPos,
  rcvVel,
  rcvXYZ,
  rcvHeading,
  rcvImu,
  rcvOdom,
};

enum {
  offset_head_sync = 0,
  offset_head_lenth = 3,
  offset_msg_id,
  offset_msg_type = 6,
  offset_msg_lenth = 8,
  offset_idle_time = 12,
  offset_time_status = 13,
  offset_week = 14,
  offset_msCnt = 16,
  offset_bd2gps_sec = 24,
};

enum {
  msg_pos_id = 42,
  msg_vel_id = 99,
  msg_xyz_id = 241,
  msg_heading_id = 971,
  msg_imu_id = 1461,
};

typedef struct _T_User_RcvMgr {
  unsigned char Buffer[RECV_BUFFER_SIZE];
  unsigned int idx;
  unsigned int rcvHeadLenth;
  unsigned int rcvLen;
  unsigned short state;
  unsigned short msgType;
  unsigned int ms_count;
} T_User_RcvMgr;

typedef struct _T_User_FrameHead {
  unsigned char head[4];
  unsigned int state;
} T_User_FrameHead;

#pragma pack(4)
typedef struct {
  unsigned char error;         // 1
  unsigned char type;          // 1
  unsigned short int GpsWeek;  // 2
  double second_week;          // 8
  unsigned int status;         // 4
  int acce_z;                  // 4
  int acce_y;                  // 4
  int acce_x;                  // 4
  int gyro_z;                  // 4
  int gyro_y;                  // 4
  int gyro_x;                  // 4  total: 40 byte
  unsigned short Week;         // week 4
  unsigned int GpsTime;        // ms  4
  unsigned long int GpsTime_pst;
} RawImuOri;
#pragma pack()

typedef struct {
  char sync_1;                //	sync :		1
  char sync_2;                //				1
  char sync_3;                //				1
  unsigned char headLen;      //	head len	1
  unsigned short int ID;      //	msgID		2
  char Type;                  //	msgType		1
  unsigned char Res1;         //	res1		1
  unsigned short int msgLen;  //	msgLen;		2
  unsigned short int Res2;    //	res2		2
  unsigned char Idle;         //	Idle		1
  unsigned char Status;       //	Status		1
  unsigned short int Week;    //	week		2
  unsigned int GpsTime;       //	ms			4
  unsigned int Res3;          //	res3		4
  unsigned short int BD2;     //	bd2			2
  unsigned short int Res4;    //	res4		2   total: 28

} RawHeader;

typedef struct {
  unsigned int sol_status;     // 4
  unsigned int pos_type;       // 4
  float base_length;           // 4
  float heading;               // 4
  float pitch;                 // 4
  float res1;                  // 4
  float hdgstddev;             // 4
  float ptchstddev;            // 4
  int stn_id;                  // 4
  unsigned char SVs;           // 1
  unsigned char solnSVs;       // 1
  unsigned char obs;           // 1
  unsigned char multi;         // 1
  unsigned char res2;          // 1
  unsigned char ext_sol_stat;  // 1
  unsigned char res3;          // 1
  unsigned char sig_mask;      // 1 total: 44 byte
  unsigned short Week;         // week
  unsigned int GpsTime;        // ms
  unsigned int GpsTime_pst;
} RawGnssYaw;

typedef struct {
  unsigned int sol_status;     // 4
  unsigned int pos_type;       // 4
  double latitude;             // 8
  double longitude;            // 8
  double altitude;             // 8
  float undulation;            // 4
  unsigned int datum_id;       // 4
  float lat_std;               // 4
  float lon_std;               // 4
  float alt_std;               // 4
  unsigned int stn_id;         // 4
  float diff_age;              // 4
  float sol_age;               // 4
  unsigned char SVs;           // 1
  unsigned char solnSVs;       // 1
  unsigned char res1;          // 1
  unsigned char res2;          // 1
  unsigned char res3;          // 1
  unsigned char ext_sol_stat;  // 1
  unsigned char res4;          // 1
  unsigned char sig_mask;      // 1 total:72 byte
  unsigned short Week;         // week
  unsigned int GpsTime;        // ms
  unsigned int GpsTime_pst;
} RawGnssPos;

typedef struct {
  unsigned int sol_status;  // 4
  unsigned int vel_type;    // 4
  float latency;            // 4
  float age;                // 4
  double hor_spd;           // 8 对地速度
  double trk_gnd;           // 8
  double vert_spd;          // 8
  unsigned int res;         // 4	total: 44 byte
  unsigned short Week;      // week
  unsigned int GpsTime;     // ms
  unsigned int GpsTime_pst;
} RawGnssVel;

#pragma pack(2)
typedef struct {
  unsigned int pos_sol_status;  // 4
  unsigned int pos_type;        // 4
  double p_x;                   // 8
  double p_y;                   // 8
  double p_z;                   // 8
  float px_std;                 // 4
  float py_std;                 // 4
  float pz_std;                 // 4	44 byte
  unsigned int vel_sol_status;  // 4
  unsigned int vel_type;        // 4
  double v_x;                   // 8
  double v_y;                   // 8
  double v_z;                   // 8
  float vx_std;                 // 4
  float vy_std;                 // 4
  float vz_std;                 // 4	44 byte
  unsigned int stn_id;          // 4
  float vel_latency;            // 4
  float diff_age;               // 4
  float sol_age;                // 4
  unsigned char SVs;            // 1
  unsigned char solnSVs;        // 1
  unsigned char ggL1;           // 1
  unsigned char solnMultiSVs;   // 1
  unsigned char res1;           // 1
  unsigned char ext_sol_stat;   // 1
  unsigned char Gsig_mask;      // 1
  unsigned char CGBsig_mask;    // 1 total:112 byte
  unsigned short Week;          // week
  unsigned int GpsTime;         // ms
  unsigned int GpsTime_pst;
} RawGnssXYZ;
#pragma pack()

#pragma pack(2)
typedef struct {
  unsigned char sync_1;   //	sync :		1
  unsigned char sync_2;   //				1
  unsigned char sync_3;   //				1
  unsigned char headLen;  //	head len	1

} OdomHeader;

typedef struct {
  unsigned short int weeknum;  // 2
  unsigned int GpsTime;        // 4
  unsigned int left_speed;     // 4
  unsigned int right_speed;    // 4
  unsigned int GpsTime_pst;

} RawOdomVel;
#pragma pack()

#endif
