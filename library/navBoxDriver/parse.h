#ifndef __PARSE_H
#define __PARSE_H

#define kGRAVITY 9.80665            // g
#define kDEG2RAD 0.017453292519943  // deg -> rad
#define kPI 3.141592653589793       // pi
#define myAE 6378137
#define myAP 6356755

// sol type
#define SOL_COMPUTED 0      // 已解出
#define INSUFFICIENT_OBS 1  // 观测数据不足
#define NO_CONVERGENCE 2    // 无法收敛
#define COV_TRACE 4         // 协方差矩阵的迹超过最大值

// pos & vel type
#define NONE 0              // 无解
#define FIXEDPOS 1          // 位置由 FIX POSITION 命令指定
#define FIXEDHEIGHT 2       // 暂不支持
#define DOPPLER_VELOCITY 8  // 速度由即时多普勒信息导出
#define SINGLE 16           // 单点定位
#define PSRDIFF 17          // 伪距差分解
#define WAAS 18             // SBAS 定位
#define L1_FLOAT 32         // L1 浮点解
#define IONOFREE_FLOAT 33   // 消电离层浮点解
#define NARROW_FLOAT 34     // 窄巷浮点解
#define L1_INT 48           // L1 固定解
#define WIDE_INT 49         // 宽巷固定解
#define NARROW_INT 50       // 窄巷固定解
#define INS 52              // 纯惯导定位解
#define INS_PSRSP 53        // 惯导与单点定位组合解
#define INS_PSRDIFF 54      // 惯导与伪距差分定位组合解
#define INS_RTKFLOA 55      // 惯导与载波相位差分浮点解组合解
#define INS_RTKFIXED 56     // 惯导与载波相位差分固定解组合解

#endif
