#pragma once

#include <string>
#include <mutex>

#include "locator.h"


using namespace std;

/**
 * BrainData 类，记录 Brain 在决策中需要用到的所在数据
 * 现在未考虑多线程读写的问题，后续看是否需要添加
 */
class BrainData
{
public:
    rclcpp::Time lastSuccessfulLocalizeTime;

    // /* ------------------------------------ 球赛相关状态量 ------------------------------------ */
    int lastScore = 0;
    int penalty[4];                                                                        // 所有机器人 penalty 状态

    // /* ------------------------------------ 数据记录 ------------------------------------ */
    // // 身体位置 & 速度指令
    Pose2D robotPoseToOdom;  // 机器人在 Odom 坐标系中的 Pose, 通过 odomCallback 更新数据
    Pose2D odomToField;      // Odom 坐标系原点在 Field 坐标系中的位置和方向.  可通过已知位置进行校准, 例如上场时根据上场点校准
    Pose2D robotPoseToField; // 机器人当前在球场坐标系中的位置和方向. 球场中心为原点, x 轴指向对方球门(前方), y 轴指向左方. 逆时针为 theta 正方向.


    // // 头部位置 通过 lowStateCallback 更新数据
    double headPitch; // 当前头部的 pitch, 单位 rad, 0 点为水平向前, 向下为正.
    double headYaw;   // 当前头部的 yaw, 单位 rad, 0 点为向前, 向左为正.

    // // 足球
    bool ballDetected = false;                               // 当前摄像头是否识别到了足球
    GameObject ball;                                         // 球的各项信息记录, 包含位置, boundingbox 等
    double robotBallAngleToField;                            // 机器人到球的向量, 在球场坐标系中与 X 轴的夹角, (-PI,PI]

    // // 场上其它对象
    vector<GameObject> opponents = {}; // 对方球员的各项信息记录, 包含位置, boundingbox 等
    vector<GameObject> goalposts = {}; // 球门柱各项信息记录, 包含位置, boundingbox 等
    vector<GameObject> markings = {};  // 球场上的标记交叉点

    // // 运动规划
    double dribbleTargetAngle;    // 带球的方向
    bool dribbleTargetAngleFound; // 带球方向规划是否成功
    double moveTargetAngle;       // 行走的目标方向

    // 封装的一些功能性小函数
    vector<FieldMarker> getMarkers();
    // 将一个 Pose 从 robot 坐标系转到 field 坐标系
    Pose2D robot2field(const Pose2D &poseToRobot);
    // 将一个 Pose 从 field 坐标系转到 robot 坐标系
    Pose2D field2robot(const Pose2D &poseToField);
};