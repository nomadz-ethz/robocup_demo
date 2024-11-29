#pragma once

#include <iostream>
#include <string>
#include <rerun.hpp>

#include <booster_msgs/msg/rpc_req_msg.hpp>


using namespace std;

class Brain; // 类相互依赖，向前声明


/**
 * RobotClient 类，调用 RobotSDK 操控机器人的操作都放在这里
 * 因为目前的代码里依赖 brain 里相关的一些东西，现在设计成跟 brain 相互依赖
 */
class RobotClient
{
public:
    RobotClient(Brain* argBrain) : brain(argBrain) {}

    void init();

    /**
     * @brief 移动机器人的头部
     *
     * @param pitch
     * @param yaw
     *
     * @return int , 0 表示执行成功
     */
    int moveHead(double pitch, double yaw);

    /**
     * @brief 设置机器人的移动速度
     * 
     * @param x double, 向前(m/s)
     * @param y double, 向左(m/s)
     * @param theta double, 逆时针转动角度(rad/s)
     * @param applyMinX, applyMinY, applyMinTheta bool 当速度指令过小时, 是否调整指令大小以防止不响应.
     * 
     * @return int , 0 表示执行成功
     * 
    */
    int setVelocity(double x, double y, double theta, bool applyMinX=true, bool applyMinY=true, bool applyMinTheta=true);

    /**
     * @brief 以速度模式走向球场坐标系中的某个 Pose, 注意最后朝向也要达到
     * 
     * @param tx, ty, ttheta double, 目标的 Pose, Field 坐标系
     * @param longRangeThreshold double, 距离超过这个值时, 优先转向目标点走过去, 而不是直接调整位置
     * @param turnThreshold double, 目标点所在的方向(注意不是最终朝向 ttheta) 与目前的角度相差大于这个值时, 先转身朝向目标
     * @param vxLimit, vyLimit, vthetaLimit double, 各方向速度的上限, m/s, rad/s
     * @param xTolerance, yTolerance, thetaTolerance double, 判断已经到达目标点的容差
     * 
     * @return int 运控命令返回值, 0 代表成功
     */
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance);

    /**
     * @brief 挥手
     */
    int waveHand(bool doWaveHand);

private:
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
};