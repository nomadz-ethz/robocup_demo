#pragma once

#include <string>
#include <ostream>

#include "types.h"
#include "utils/math.h"

using namespace std;

/**
 * 存储 Brain 需要的一些配置值，这些值应该是初始化就确认好了，在机器人决策过程中只读不改的
 * 需要在决策过程中变化的值，应该放到 BrainData 中
 * 注意：
 * 1、配置文件会从 config/config.yaml 中读取
 * 2、如果存在 config/config_local.yaml，将会覆盖 config/config.yaml 的值
 * 
 */
class BrainConfig
{
public:
    // ---------- start config from config.yaml ---------------------------------------------
    // 这部分的变量，是直接从配置文件中读取到的原始值，如果在配置文件中新增了配置，在这里添加对应的变量存储
    // 这些值会在 BrainNode 中被覆盖，所以即使 config.yaml 中没显示配置，这里的默认值不会生效。
    // 真正要设置默认值得在 BrainNode 的 declare_parameter 里配置
    int teamId;                 // 对应 game.team_id
    int playerId;               // 对应 game.player_id
    string fieldType;           // 对应 game.field_type  球场类型, "adult_size"(14*9) | "kid_size" (9*6)
    string playerRole;          // 对应 game.player_role   "striker" | "goal_keeper"
    string playerStartPos;      // 对应 game.player_start_post  "left" | "right", 从自己半场的左侧还是右侧上场
    
    double robotHeight;         // 对应 robot.robot_height 机器人的身高(m), 用于估算距离, 可以通过 SetParam 节点进行调试. In behaviortree xml: <SetParam code="robot_height=1.1" />
    double robotOdomFactor;     // 对应 robot.odom_factor odom 认为走的距离与机器人实际走的距离的比值, 用于修正 odom
    double vxFactor;            // 对应 robot.vx_factor 修正 vx 实际比指令大的问题
    double yawOffset;           // 对应 robot.yaw_offset 修正测距时往左偏的问题 
    
    bool rerunLogEnable;        // 对应 rerunLog.enable  是否开启 rerunLog
    string rerunLogServerAddr;    // 对应 rerunLog.server_ip  rerunLog 服务器 IP
    int rerunLogImgInterval;    // 对应 rerunLog.img_interval 每多少次消息记录一次 log 一次 img, 提升这一值可以减少 log 大小, 提升 log 传输速度.
    
    string treeFilePath;        // 现在没放在 config.yaml 中了，放在 launch.py 中指定，行为树文件的路径
    // ----------  end config from config.yaml ---------------------------------------------

    // game 参数
    FieldDimensions fieldDimensions; // 球场尺寸

    // 相机像素
    double camPixX = 1280;
    double camPixY = 720;

    // 相机视角
    double camAngleX = deg2rad(90);
    double camAngleY = deg2rad(65);

    // 头转动软限位
    double headYawLimitLeft = 1.1;
    double headYawLimitRight = -1.1;
    double headPitchLimitUp = 0.0;

    // 速度上限
    double vxLimit = 1.2;
    double vyLimit = 0.4;
    double vthetaLimit = 1.5;

    // 策略参数
    double safeDist = 2.0;                  // 用于碰撞检测的安全距离, 小于这个距离认为碰撞
    double goalPostMargin = 0.4;
    double goalPostMarginForTouch = 0.1; // 计算球门柱的margin，用于计算球门柱的角度，越大则计算时候球门越小，touch的时候，这个margin不一样，通常会更小
    double memoryLength = 3.0;           // 连续多少秒看不到球时, 就认为球丢了

    // BrainNode 填充完参数后，调用 handle() 进行一些参数的处理（校正、计算等）,成功返回 true
    void handle();
    
    // 把配置信息输出到指定的输出流中(debug用)
    void print(ostream &os);
};