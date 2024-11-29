#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();

    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value = T();
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;

    /**
     * 初始化 blackboard 里的 entry，注意新加字段，在这里设置个默认值
     */
    void initEntry();
};

// ------------------------------- 比赛用 -------------------------------

// Striker 的比赛决策, 决定 Striker 在比赛中什么时候执行哪种技术动作
class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            InputPort<string>("position", "offense", "offense | defense, 决定了向哪个方向踢球"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// Goal-keeper 的比赛决策, 决定其在比赛中什么时候执行哪种技术动作
class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "小于这个角度, 认为 adjust 已经成功"),
            InputPort<double>("adjust_y_tolerance", 0.1, "y 方向偏移小于这个值, 认为 y 方向 adjust 成功"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

// CamTrackBall, 摄像头跟随球运行, 保持球位于画面的中心
class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override;

private:
    Brain *brain;
};

// CamFindBall, 没看到球时，尝试找到球. 注意, 之所以没有做成 Stateful Node, 是因为 CamFindBall 后通常还要执行其它动作
class CamFindBall : public SyncActionNode
{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;

private:
    double _cmdSequence[6][2];    // 找球的动作序列， 依次看向这几个位置
    rclcpp::Time _timeLastCmd;    // 上一次执行命令的时间，用于确保命令之间具有时间间隔
    int _cmdIndex;                // 当前执行到 cmdSequence 中的哪一步了
    long _cmdIntervalMSec;        // 执行动作序列的时间间隔，单位毫秒
    long _cmdRestartIntervalMSec; // 距离上一次执行超过这个时间，则重新从 0 开始执行序列

    Brain *brain;
    // TODO, 暴露这些参数
};

// 机器人执行找球的动作, 需要与 CamFindBall 配合使用
class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "转向的速度上限"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _turnDir; // 1.0 向左 -1.0 向右
    Brain *brain;
};

// 追球, 如果球在自己的后面, 会绕到球的后面
class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.4, "追球的最大 x 速度"),
            InputPort<double>("vy_limit", 0.4, "追球的最大 y 速度"),
            InputPort<double>("vtheta_limit", 0.1, "追球时, 实时调整方向的速度不大于这个值"),
            InputPort<double>("dist", 1.0, "追球的目标是球后面多少距离"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     // circl_back, chase;
    double _dir = 1.0; // 1.0 从右侧 circle back, -1.0 从右侧 circle back
};

// 已经接近球后, 调整到合适进攻或防御的踢球角度
class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.2, "球的角度大于这个值, 机器人先转身面向球"),
            InputPort<double>("vx_limit", 0.1, "调整过过程中 vx 的限制 [-limit, limit]"),
            InputPort<double>("vy_limit", 0.1, "调整过过程中 vy 的限制 [-limit, limit]"),
            InputPort<double>("vtheta_limit", 0.4, "调整过过程中 vtheta 的限制 [-limit, limit]"),
            InputPort<double>("max_range", 1.5, "ball  range 超过这个值时, 向前一点"),
            InputPort<double>("min_range", 1.0, "ball range 小于这个值时, 后退一点"),
            InputPort<string>("position", "offense", "offense | defense, 决定了向哪个方向踢球")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// 执行踢球动作
class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("min_msec_kick", 500, "踢球动作最少执行多少毫秒"),
            InputPort<int>("msec_stand", 500, "发出停止指令后, 多少毫秒后认为已经站定"),
            InputPort<double>("vx_limit", 1.2, "vx 最大值"),
            InputPort<double>("vy_limit", 0.4, "vy 最大值"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; // 开始时间
    int _msecKick = 1000;    // 在开始时, 根据距离估算执行踢球动作的的持续时间
};

// CamScanField, 视角画圈扫视, 先抬头向一个方向, 再低头向另一个方向扫视, 此为一圈
class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("low_pitch", 0.4, "向下看时的最大 pitch"),
            InputPort<double>("high_pitch", 0.2, "向上看时的最小 pitch"),
            InputPort<double>("left_yaw", 0.8, "向左看时的最大 yaw"),
            InputPort<double>("right_yaw", -0.8, "向右看时的最小 yaw"),
            InputPort<int>("msec_cycle", 4000, "多少毫秒转一圈"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// SelfLocate, 利用粒子滤波对当前的位置进行校正, 纠正里程计的漂移
class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("mode", "enter_field", "must be one of [enter_field, trust_direction, trust_position, trust_nothing, face_forward]"),
            // enter_field: 上场时使用，此时必然在已方半场，且可以根据已方球门的位置进一步缩小方向范围
            // trust_direction: 正常情况下使用，此时 odom 信息大体上是准确的（未摔倒过）
            // trust_position: 使用于摔倒后，此时 x,y 可信，但方向不可信。（注意，如果此时在中线附近，则因为球场的对称性，需认为位置也不可信）
            // trust_nothing: 极限情况，认为 x,y 也不可信，需要先通过标志物辨别方向。
            // face_forward: 面向对方球门的方向, 主要用于测试
        };
    };

private:
    Brain *brain;
};

// 移动到 Field 坐标系中的一个 Pose, 包含最终目标方向. 最好与 CamScanField 和 SelfLocate 同时使用, 以使最终位置较为准确
class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "目标 x 坐标, Field 坐标系"),
            InputPort<double>("y", 0, "目标 y 坐标, Field 坐标系"),
            InputPort<double>("theta", 0, "目标最终朝向, Field 坐标系"),
            InputPort<double>("long_range_threshold", 1.5, "目标点的距离超过这个值时, 优先走过去, 而不是细调位置和方向"),
            InputPort<double>("turn_threshold", 0.4, "长距离时, 目标点的方向超这个数值时, 先转向目标点"),
            InputPort<double>("vx_limit", 1.0, "x 限速"),
            InputPort<double>("vy_limit", 0.5, "y 限速"),
            InputPort<double>("vtheta_limit", 0.4, "theta 限速"),
            InputPort<double>("x_tolerance", 0.2, "x 容差"),
            InputPort<double>("y_tolerance", 0.2, "y 容差"),
            InputPort<double>("theta_tolerance", 0.1, "theta 容差"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

/**
 * @brief 设置机器人的速度
 *
 * @param x,y,theta double, 机器人在 x，y 方向上的速度（m/s）和逆时针转动的角速度（rad/s), 默认值为 0. 全为 0 时，即相当于给出站立不动指令
 *
 */
class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("action", "start", "start | stop"),
        };
    }

private:
    Brain *brain;
};

// ------------------------------- FOR DEMO -------------------------------

// 用于演示跟着球跑, 不是比赛时使用的结点. 与 Chase 不同在于, Simple Chase 只是不断向球走, 而不会绕到球背后, 也因此不需要球场\定位\视频也可以运行
class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "在距离球多远的距离, 就不再走向球了"),
            InputPort<double>("stop_angle", 0.1, "球的角度在多少时, 就不再转向球了"),
            InputPort<double>("vy_limit", 0.2, "限制 Y 方向速度, 以防止走路不稳定. 要起作用需要小于机器本身的限速 0.4"),
            InputPort<double>("vx_limit", 0.6, "限制 X 方向速度, 以防止走路不稳定. 要起作用需要小于机器本身的限速 1.2"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// ------------------------------- FOR DEBUG -------------------------------

// 向 cout 打印文字
class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("msg")};
    }

private:
    Brain *brain;
};
