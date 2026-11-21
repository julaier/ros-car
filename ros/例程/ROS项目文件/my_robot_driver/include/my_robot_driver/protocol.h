#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <vector>
#pragma pack(1) // 必须强制 1 字节对齐

#define HEADER_1 0x55
#define HEADER_2 0xAA
#define ID_ROS2CTR_VEL 0x01  // 树莓派 -> STM32
#define ID_CTR2ROS_DATA 0x02 // STM32 -> 树莓派

// 树莓派下发给 STM32 的数据
struct ControlData
{
    int16_t vx;  //沿x轴方向速度
    int16_t vy; // 沿y轴方向速度
    int16_t wz; //车头旋转的速度
    int16_t yaw;// 移动的方向，但是无
}; 
// STM32 上传给树莓派的数据
struct FeedbackData
{
    int16_t vx;  // 沿x轴方向速度
    int16_t vy;  //// 沿y轴方向速度
    int16_t wz;  // 车头旋转的速度
    int16_t yaw; // IMU 偏航角 * 1000 (rad)
                 // 如果需要 IMU 原始数据，可在此扩展 ax, ay, az...
};
#pragma pack()
// 辅助工具类：处理校验
class ProtocolHelper 
{
    public:    
    static uint8_t calculateChecksum(uint8_t* buf, int len);
};

#endif