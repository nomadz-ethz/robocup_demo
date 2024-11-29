/**
 * @file types.h
 * @brief 定义 brain 项目中用到的 struct 及 enum
 */
#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>
#include <rclcpp/rclcpp.hpp>

using namespace std;

/* ------------------ Struct ------------------------*/

// 球场尺寸信息
struct FieldDimensions
{
    double length;            // 球场长度
    double width;             // 球场宽度
    double penaltyDist;       // 罚球点距离底线的直线距离
    double goalWidth;         // 球门的宽度
    double circleRadius;      // 中圈的半径
    double penaltyAreaLength; // 禁区的长
    double penaltyAreaWidth;  // 禁区的宽
    double goalAreaLength;    // 球门区的长
    double goalAreaWidth;     // 球门区的宽
                              // 注意: 禁区比球门区大；禁区和球门区的长与宽实际上要小。这个命名是为了与比赛规则相统一。
};
const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
const FieldDimensions FD_ADULTSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};

// Pose2D, 记录平面上的一个点以及朝向
struct Pose2D
{
    double x = 0;
    double y = 0;
    double theta = 0; // rad, 从 x 轴正方向开始, 逆时针为正
};

// Point, 记录一个三维点
struct Point
{
    double x;
    double y;
    double z;
};

// Point2D, 记录一个二维点
struct Point2D
{
    double x;
    double y;
};

// BoundingBox
struct BoundingBox
{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
};

// GameObject, 用于存储比赛中的重要实体信息，如 Ball, Goalpost 等。相比于 /detect 消息中的 detection::DetectedObject，它的信息更为丰富。
struct GameObject
{
    // --- 从 /detect 消息中获得 ---
    string label;              // 物体被识别为什么
    BoundingBox boundingBox;   // 物体在摄像头中的识别框, 左上角为 0 点, 向右为 x, 向下为 y
    Point2D precisePixelPoint; // 物体的精确像素点位置, 仅地面标志点有这一数据
    double confidence;         // 识别的置信度
    Point posToRobot;          // 物体在机器人本体坐标系的的位置, 位置为 2D, 忽略 z 值.

    // --- 在 processDetectedObject 函数中计算获得 ---
    string info;                     // 用于存储额外的信息, 如, 门柱对象可以存储门柱是哪根门柱
    Point posToField;                // 物体在物体场坐标系的的位置, 位置为 2D, 忽略 z 值. x 向前, y 向左.
    double range;                    // 物体距离机器人中心在物体场平面上的投影点的直线距离
    double pitchToRobot, yawToRobot; // 物体相对于机器人正前方的 pitch 和 yaw, 单位 rad, 向下和向左为正
    rclcpp::Time timePoint;          // 物体被检测到的时间
};

// Joystick 按键对应的数字
enum JoystickBTN
{
    BTN_X,
    BTN_A,
    BTN_B,
    BTN_Y,
    BTN_LB,
    BTN_RB,
    BTN_LT,
    BTN_RT,
    BTN_BACK,
    BTN_START,
};

enum JoystickAX
{
    AX_LX,
    AX_LY,
    AX_RX,
    AX_RY,
    AX_DX,
    AX_DY,
};