#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vision_interface/msg/detections.hpp>
#include <game_controller_interface/msg/game_control_data.hpp>
#include <booster/robot/b1/b1_api_const.hpp>

#include "booster_interface/msg/odometer.hpp"
#include "booster_interface/msg/low_state.hpp"

#include "brain_config.h"
#include "brain_data.h"
#include "brain_log.h"
#include "brain_tree.h"
#include "locator.h"
#include "robot_client.h"

using namespace std;

/**
 * Brain 核心类，因为要传递智能指针给 BrainTree，所以需要继承自 enable_shared_from_this
 * 数据封闭到各个子对象中，不要直接存在在 Brain 类
 * 如果是静态的配置值，放到 config
 * 运行时的动态数据，放到 data
 * TODO:
 * BehaviorTree 的 blackboard 里也存了一些数据，看看是不是有些重复存储了的，可以考虑去掉,
 * 目前的设计里，brain 指针传到了 BehaviorTree 的结点里了，在那里直接访问 brain->config 和 brain->data
 */
class Brain : public rclcpp::Node
{
public:
    // BrainConfig 对象，主要包含运行时需要的配置值（静态）
    std::shared_ptr<BrainConfig> config;
    // BrainLog 对象，封装 rerun log 相关的操作
    std::shared_ptr<BrainLog> log;
    // BrainData 对象，Brain 所有运行时的值都放在这里
    std::shared_ptr<BrainData> data;
    // RobotClient 对象，包含所有对机器人的操作
    std::shared_ptr<RobotClient> client;
    // locator 对象
    std::shared_ptr<Locator> locator;
    // BrainTree 对象，里面包含 BehaviorTree 相关的操作
    std::shared_ptr<BrainTree> tree;

    // 构造函数，接受 nodeName 创建 ros2 结点
    Brain();

    // 初始化操作，只需要在 main 函数中调用一次，初始化过程如不符合预期，可以抛异常直接中止程序
    void init();

    // 在 ros2 循环中调用
    void tick();

    /**
     * @brief 计算当前球到对方两个球门柱的向量角度, 球场坐标系
     *
     * @param  margin double, 计算角度时, 返回值比实际的球门的位置向内移动这个距离, 因为这个角度球会被门柱挡住. 此值越大, 射门准确性越高, 但调整角度花的时间也越长.
     *
     * @return vector<double> 返回值中 vec[0] 是左门柱, vec[1] 是右门柱. 注意左右都是以对方的方向为前方.
     */
    vector<double> getGoalPostAngles(const double margin = 0.3);

    // 根据真值校准 odom. 参数 x, y, theta 代表在球场坐标系的机器人的 Pose 真值
    void calibrateOdom(double x, double y, double theta);

    double msecsSince(rclcpp::Time time);

private:
    void loadConfig();

    // 看不见球时, 可以利用记忆中球在 Field 中的位置以及机器人 Odom 信息更新球的相对位置
    void updateBallMemory();

    // 处理记忆相关的逻辑, 例如多久看不到球时, 就可以认为记忆中的球的位置已经不可信了. 或没有看到球时, 根据记忆更新球相对于机器人的位置.
    void updateMemory();

    // ------------------------------------------------------ SUB CALLBACKS ------------------------------------------------------

    // 需要使用手柄控制机器人进行调试时, 可以在这里获得手柄的按键状态.
    // 注意: 启动手柄节点时, 需要给秋 autorepeate_rate 参数为 0.0, 即:  ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0
    void joystickCallback(const sensor_msgs::msg::Joy &msg);

    // 用于接收并处理裁判上位机的消息
    void gameControlCallback(const game_controller_interface::msg::GameControlData &msg);
    // 处理视觉识别消息
    void detectionsCallback(const vision_interface::msg::Detections &msg);

    // 处理摄像头图像信息, 这里仅用于将图片记录入日志
    void imageCallback(const sensor_msgs::msg::Image &msg);

    // 处理里程计消息
    void odometerCallback(const booster_interface::msg::Odometer &msg);

    // 处理底层状态信息, 如 imu 以及头部关节状态消息 (用以计算摄像头角度)
    void lowStateCallback(const booster_interface::msg::LowState &msg);

    // 处理头部相对于双脚坐标系的信息 (用以debug)
    void headPoseCallback(const geometry_msgs::msg::Pose &msg);

    // ------------------------------------------------------ 视觉处理 ------------------------------------------------------
    // 将 detection 中的消息进行加工丰富, 获得更完整的对象信息. 例如计算 Field 坐标系中的位置, 相对于机器人的角度等
    vector<GameObject> getGameObjects(const vision_interface::msg::Detections &msg);
    // 进一步处理检测到的足球
    void detectProcessBalls(const vector<GameObject> &ballObjs);

    // 进一步处理检测到的场地标志点
    void detectProcessMarkings(const vector<GameObject> &markingObjs);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<game_controller_interface::msg::GameControlData>::SharedPtr gameControlSubscription;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odometerSubscription;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr headPoseSubscription;
};
