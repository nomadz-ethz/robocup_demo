#include <iostream>
#include <string>

#include "brain.h"
#include "utils/print.h"
#include "utils/math.h"

using namespace std;
using std::placeholders::_1;

Brain::Brain() : rclcpp::Node("brain_node")
{
    // 要注意参数必须先在这里声明，否则程序里也读不到
    // 配置在 yaml 文件中的参数，如果有层级结构，用点分号来获取

    declare_parameter<int>("game.team_id", 0);
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.player_start_pos", "");

    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.95);
    declare_parameter<double>("robot.yaw_offset", 0.1);

    declare_parameter<bool>("rerunLog.enable", false);
    declare_parameter<string>("rerunLog.server_addr", "");
    declare_parameter<int>("rerunLog.img_interval", 10);

    // tree_file_path 在 launch.py 中配置的，没有放在 config.yaml 中
    declare_parameter<string>("tree_file_path", "");
}

void Brain::init()
{
    // 一定要先加载配置，之后 config 才能使用
    config = std::make_shared<BrainConfig>();
    loadConfig();

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();

    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);

    // 初始化粒子滤波定位器
    locator->init(config->fieldDimensions, 4, 0.5);

    // 构建 BehaviorTree
    tree->init();

    // 初始化 client
    client->init();

    log->prepare();
    // 初始化上次定位成功的时间. 这个时间用于估计定位时里程计的最大漂移量

    data->lastSuccessfulLocalizeTime = get_clock()->now();

    // 创建各个 subscription
    joySubscription = create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bind(&Brain::joystickCallback, this, _1));
    gameControlSubscription = create_subscription<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 1, bind(&Brain::gameControlCallback, this, _1));
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", 1, bind(&Brain::detectionsCallback, this, _1));
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state", 1, bind(&Brain::odometerCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", 1, bind(&Brain::lowStateCallback, this, _1));
    imageSubscription = create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 1, bind(&Brain::imageCallback, this, _1));
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose", 1, bind(&Brain::headPoseCallback, this, _1));
}

void Brain::loadConfig()
{
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.player_start_pos", config->playerStartPos);

    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("robot.odom_factor", config->robotOdomFactor);
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);

    get_parameter("rerunLog.enable", config->rerunLogEnable);
    get_parameter("rerunLog.server_addr", config->rerunLogServerAddr);
    get_parameter("rerunLog.img_interval", config->rerunLogImgInterval);

    get_parameter("tree_file_path", config->treeFilePath);

    // 参数非法会直接抛异常导致程序结束运行
    config->handle();

    // 加载完参数，打印
    ostringstream oss;
    config->print(oss);
    prtDebug(oss.str());
}

/**
 * 会被循环调用的 tick 函数
 */
void Brain::tick()
{
    updateMemory();
    tree->tick();
}

void Brain::updateMemory()
{
    updateBallMemory();

    // 处理对方 kickoff 时, 我方的等待逻辑
    static Point ballPos;
    static rclcpp::Time kickOffTime;
    if (
        tree->getEntry<string>("player_role") == "striker" && ((tree->getEntry<string>("gc_game_state") == "SET" && !tree->getEntry<bool>("gc_is_kickoff_side")) || (tree->getEntry<string>("gc_game_sub_state") == "SET" && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side"))))
    {
        ballPos = data->ball.posToRobot;
        kickOffTime = get_clock()->now();
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    }
    else if (tree->getEntry<bool>("wait_for_opponent_kickoff"))
    {
        if (
            norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y) > 0.3 || (get_clock()->now() - kickOffTime).seconds() > 10.0)
        {
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }
}

void Brain::updateBallMemory()
{
    // update Pose to field from Pose to robot (based on odom)
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(data->robotPoseToField.theta) * data->robotPoseToField.x - cos(data->robotPoseToField.theta) * data->robotPoseToField.y;
    xfr = -cos(data->robotPoseToField.theta) * data->robotPoseToField.x - sin(data->robotPoseToField.theta) * data->robotPoseToField.y;
    thetafr = -data->robotPoseToField.theta;
    transCoord(
        data->ball.posToField.x, data->ball.posToField.y, 0,
        xfr, yfr, thetafr,
        data->ball.posToRobot.x, data->ball.posToRobot.y, data->ball.posToRobot.z // 注意, z 没有在其它地方使用, 这里仅为参数点位使用
    );

    data->ball.range = sqrt(data->ball.posToRobot.x * data->ball.posToRobot.x + data->ball.posToRobot.y * data->ball.posToRobot.y);
    tree->setEntry<double>("ball_range", data->ball.range);
    data->ball.yawToRobot = atan2(data->ball.posToRobot.y, data->ball.posToRobot.x);
    data->ball.pitchToRobot = asin(config->robotHeight / data->ball.range);

    // mark ball as lost if long time no see
    if (get_clock()->now().seconds() - data->ball.timePoint.seconds() > config->memoryLength)
    { // 上次检测到球的时间过长, 则认为记忆中的球失效了
        tree->setEntry<bool>("ball_location_known", false);
        data->ballDetected = false;
    }

    // log mem ball pos
    log->setTimeNow();
    log->log("field/memball",
             rerun::LineStrips2D({
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x - 0.2, -data->ball.posToField.y}, {data->ball.posToField.x + 0.2, -data->ball.posToField.y}},
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x, -data->ball.posToField.y - 0.2}, {data->ball.posToField.x, -data->ball.posToField.y + 0.2}},
                                 })
                 .with_colors({tree->getEntry<bool>("ball_location_known") ? 0xFFFFFFFF : 0xFF0000FF})
                 .with_radii({0.005})
                 .with_draw_order(30));
}

vector<double> Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY; // 球门柱在球场中的坐标

    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;

    // 如果看到了对方球门, 则使用看到的位置, 可以抵消 odom 的误差
    for (int i = 0; i < data->goalposts.size(); i++)
    {
        auto post = data->goalposts[i];
        if (post.info == "oppo-left")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.info == "oppo-right")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }

    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);

    vector<double> vec = {theta_l, theta_r};
    return vec;
}

void Brain::calibrateOdom(double x, double y, double theta)
{
    // TODO: 考虑时间戳问题
    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;

    // 获取 odom 在 field 坐标系中的新位置
    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);

    // 利用新的 data->odomToField 重新计算 robotToField 的值
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    // 立即在新的 odom 中更新重要的物体位置, 以防止下一次 detectcallback 之前用到的数据是错误的
    double placeHolder;
    // ball
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder // 注意,  由于物体定位没有 theta, 这里用一个 placeholder 占位
    );

    // opponents
    for (int i = 0; i < data->opponents.size(); i++)
    {
        auto obj = data->opponents[i];
        transCoord(
            obj.posToRobot.x, obj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            obj.posToField.x, obj.posToField.y, placeHolder // 注意,  由于物体定位没有 theta, 这里用一个 placeholder 占位
        );
    }
}

double Brain::msecsSince(rclcpp::Time time)
{
    return (this->get_clock()->now() - time).nanoseconds() / 1e6;
}

void Brain::joystickCallback(const sensor_msgs::msg::Joy &msg)
{
    // 通过手柄控制机器人
    if (msg.buttons[BTN_LT] == 0 && msg.buttons[BTN_RT] == 0 && msg.buttons[BTN_LB] == 0 && msg.buttons[BTN_RB] == 0)
    {
        if (msg.buttons[BTN_B] == 1)
        {
            tree->setEntry<bool>("B_pressed", true);
            prtDebug("B is pressed");
        }
        else if (msg.buttons[BTN_B] == 0)
        {
            tree->setEntry<bool>("B_pressed", false);
            prtDebug("B is released");
        }
    }
    else if (msg.buttons[BTN_LT] == 1)
    {
        // 用于在线调试参数
        if (msg.axes[AX_DX] || msg.axes[AX_DY])
        {
            config->vxFactor += 0.01 * msg.axes[AX_DX];
            config->yawOffset += 0.01 * msg.axes[AX_DY];
            prtDebug(format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset));
        }

        if (msg.buttons[BTN_X] == 1)
        {
            tree->setEntry<int>("control_state", 1);
            client->setVelocity(0., 0., 0.);
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
        }
        else if (msg.buttons[BTN_A] == 1)
        {
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
        }
        else if (msg.buttons[BTN_B] == 1)
        {
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
        }
        else if (msg.buttons[BTN_Y] == 1)
        {
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
        }
    }
}

void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    // 处理比赛的一级状态
    auto lastGameState = tree->getEntry<string>("gc_game_state"); // 比赛的一级状态
    vector<string> gameStateMap = {
        "INITIAL", // 初始化状态, 球员在场外准备
        "READY",   // 准备状态, 球员进场, 并走到自己的始发位置
        "SET",     // 停止动作, 等待裁判机发出开始比赛的指令
        "PLAY",    // 正常比赛
        "END"      // 比赛结束
    };
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    tree->setEntry<string>("gc_game_state", gameState);
    bool isKickOffSide = (msg.kick_off_team == config->teamId); // 我方是否是开球方
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);

    // 处理比赛的二级状态
    string gameSubStateType = static_cast<int>(msg.secondary_state) == 0 ? "NONE" : "FREE_KICK"; // 在 PLAY 过程中的其它状态, 暂时只考虑任意球. 在此状态下时, 选手为任意球做准备, SubState 恢复 NONE 时, 重新回到 PLAY 的正常状态中.
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};                               // STOP: 停下来; -> GET_READY: 移动到进攻或防守位置; -> SET: 站住不动
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == config->teamId); // 在二级状态下, 我方是否是开球方. 例如, 当前二级状态为任意球, 我方是否是开任意球的一方
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);

    // 找到队的信息
    game_controller_interface::msg::TeamInfo myTeamInfo;
    if (msg.teams[0].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[0];
    }
    else if (msg.teams[1].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[1];
    }
    else
    {
        // 数据包中没有包含我们的队，不应该再处理了
        prtErr("received invalid game controller message");
        return;
    }

    // 处理判罚状态. penalty[playerId] 代表我方的球员是否处于判罚状态, 处理判罚状态意味着不能移动
    data->penalty[0] = static_cast<int>(myTeamInfo.players[0].penalty);
    data->penalty[1] = static_cast<int>(myTeamInfo.players[1].penalty);
    data->penalty[2] = static_cast<int>(myTeamInfo.players[2].penalty);
    data->penalty[3] = static_cast<int>(myTeamInfo.players[3].penalty);
    double isUnderPenalty = (data->penalty[config->playerId] != 0); // 当前 robot 是否被判罚中
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);

    // FOR FUN 处理进球后的庆祝挥手的逻辑
    int curScore = static_cast<int>(myTeamInfo.score);
    if (curScore > data->lastScore)
    {
        tree->setEntry<bool>("we_just_scored", true);
        data->lastScore = curScore;
    }
    if (gameState == "SET")
    {
        tree->setEntry<bool>("we_just_scored", false);
    }
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    auto gameObjects = getGameObjects(msg);

    // 对检测到的对象进行分组
    vector<GameObject> balls, goalPosts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalPosts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            // 为了调试方便, 可以在 bt 的 xml 中加入 <Script code="treat_person_as_robot:=true" />, 由人来扮演对方机器人
            if (tree->getEntry<bool>("treat_person_as_robot"))
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    detectProcessBalls(balls);
    detectProcessMarkings(markings);

    if (!log->isEnabled())
        return;

    // log detection boxes to rerun
    auto detection_time_stamp = msg.header.stamp;
    rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto now = get_clock()->now();

    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
    };

    // for logging boundingBoxes
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame

    for (int i = 0; i < gameObjects.size(); i++)
    {
        auto obj = gameObjects[i];
        auto label = obj.label;
        labels.push_back(rerun::Text(format("%s x:%.2f y:%.2f c:%.2f", obj.label.c_str(), obj.posToRobot.x, obj.posToRobot.y, obj.confidence)));
        points.push_back(rerun::Vec2D{obj.posToField.x, -obj.posToField.y}); // y 取反是因为 rerun Viewer 的坐标系是左手系。转一下看起来更方便。
        points_r.push_back(rerun::Vec2D{obj.posToRobot.x, -obj.posToRobot.y});
        mins.push_back(rerun::Vec2D{obj.boundingBox.xmin, obj.boundingBox.ymin});
        sizes.push_back(rerun::Vec2D{obj.boundingBox.xmax - obj.boundingBox.xmin, obj.boundingBox.ymax - obj.boundingBox.ymin});

        auto it = detectColorMap.find(label);
        if (it != detectColorMap.end())
        {
            colors.push_back(detectColorMap[label]);
        }
        else
        {
            colors.push_back(rerun::Color(0xFFFFFFFF));
        }
    }

    double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    log->setTimeSeconds(time);
    log->log("image/detection_boxes",
             rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                 .with_labels(labels)
                 .with_colors(colors));

    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
             // .with_labels(labels)
    );
    log->log("robotframe/detection_points",
             rerun::Points2D(points_r)
                 .with_colors(colors)
             // .with_labels(labels)
    );
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{

    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;

    // 根据 Odom 信息, 更新机器人在 Field 坐标系中的位置
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    // Log Odom 信息

    log->setTimeNow();
    log->log("field/robot",
             rerun::Points2D({{data->robotPoseToField.x, -data->robotPoseToField.y}, {data->robotPoseToField.x + 0.1 * cos(data->robotPoseToField.theta), -data->robotPoseToField.y - 0.1 * sin(data->robotPoseToField.theta)}})
                 .with_radii({0.2, 0.1})
                 .with_colors({0xFF6666FF, 0xFF0000FF}));
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;

    log->setTimeNow();

    log->log("low_state_callback/imu/rpy/roll", rerun::Scalar(msg.imu_state.rpy[0]));
    log->log("low_state_callback/imu/rpy/pitch", rerun::Scalar(msg.imu_state.rpy[1]));
    log->log("low_state_callback/imu/rpy/yaw", rerun::Scalar(msg.imu_state.rpy[2]));
    log->log("low_state_callback/imu/acc/x", rerun::Scalar(msg.imu_state.acc[0]));
    log->log("low_state_callback/imu/acc/y", rerun::Scalar(msg.imu_state.acc[1]));
    log->log("low_state_callback/imu/acc/z", rerun::Scalar(msg.imu_state.acc[2]));
    log->log("low_state_callback/imu/gyro/x", rerun::Scalar(msg.imu_state.gyro[0]));
    log->log("low_state_callback/imu/gyro/y", rerun::Scalar(msg.imu_state.gyro[1]));
    log->log("low_state_callback/imu/gyro/z", rerun::Scalar(msg.imu_state.gyro[2]));
}

void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{
    if (!config->rerunLogEnable)
        return;

    static int counter = 0;
    counter++;
    if (counter % config->rerunLogImgInterval == 0)
    {
        // 将 ROS 图像消息转换为 OpenCV 图像
        cv::Mat imageBGR(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        cv::Mat imageRGB;
        cv::cvtColor(imageBGR, imageRGB, cv::COLOR_BGR2RGB);

        // 压缩图像
        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10}; // 10 表示压缩质量，可以根据需要调整
        cv::imencode(".jpg", imageRGB, compressed_image, compression_params);

        // 将压缩后的图像数据传递给 rerun
        double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        log->setTimeSeconds(time);
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}

void Brain::headPoseCallback(const geometry_msgs::msg::Pose &msg)
{

    // --- for test: 测距 ---
    // if (config->rerunLogEnable) {
    if (false)
    {
        auto x = msg.position.x;
        auto y = msg.position.y;
        auto z = msg.position.z;

        auto orientation = msg.orientation;
        // 四元素转欧拉角
        auto roll = rad2deg(atan2(2 * (orientation.w * orientation.x + orientation.y * orientation.z), 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)));
        auto pitch = rad2deg(asin(2 * (orientation.w * orientation.y - orientation.z * orientation.x)));
        auto yaw = rad2deg(atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y), 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)));

        log->setTimeNow();

        log->log("head_to_base/text",
                 rerun::TextLog("x: " + to_string(x) + " y: " + to_string(y) + " z: " + to_string(z) + " roll: " + to_string(roll) + " pitch: " + to_string(pitch) + " yaw: " + to_string(yaw)));
        log->log("head_to_base/x",
                 rerun::Scalar(x));
        log->log("head_to_base/y",
                 rerun::Scalar(y));
        log->log("head_to_base/z",
                 rerun::Scalar(z));
        log->log("head_to_base/roll",
                 rerun::Scalar(roll));
        log->log("head_to_base/pitch",
                 rerun::Scalar(pitch));
        log->log("head_to_base/yaw",
                 rerun::Scalar(yaw));
    }
}

vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    auto timestamp = detections.header.stamp;

    rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj;

        gObj.timePoint = timePoint;
        gObj.label = obj.label;

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;

        if (obj.position.size() > 0 && !(obj.position[0] == 0 && obj.position[1] == 0))
        { // 深度测距成功， 以深度测距为准
            gObj.posToRobot.x = obj.position[0];
            gObj.posToRobot.y = obj.position[1];
        }
        else
        { // 深度测距失败，以投影距离为准
            gObj.posToRobot.x = obj.position_projection[0];
            gObj.posToRobot.y = obj.position_projection[1];
        } // 注意，z 值没有用到

        // 计算角度
        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range); // 注意这是一个近似值

        // 计算对象在 field 坐标系中的位置
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z // 注意, z 没有在其它地方使用, 这里仅为参数占位使用
        );

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    // 参数
    const double confidenceValve = 0.35;        // confidence 低于这个阈值, 认为不是球(注意detection模块目前传入目标置信度都是>0.2的)
    const double pitchLimit = deg2rad(0);       // 球相对于机器人正前方的 pitch (向下为正) 低于这个值时, 认为不是球. (因为球不会在天上)
    const int timeCountThreshold = 5;           // 连续若干帧检测到球, 才认为是球，只用于找球策略中
    const unsigned int detectCntThreshold = 3;  // 最大计数，累积如此数量帧数检测到目标才认为确实识别到了目标（目前只用于球检测）
    const unsigned int diffConfidThreshold = 4; // 追踪球和高置信度球差异次数阈值，达到后采纳高置信度的球

    double bestConfidence = 0;
    double minPixDistance = 1.e4;
    int indexRealBall = -1;  // 认为哪一个球是真的, -1 表示没有检测到球
    int indexTraceBall = -1; // 根据像素距离追踪球， -1表示没有追踪到目标

    // 找出最可能的真球
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];

        // 判断: 如果置信度太低, 则认为是误检
        if (ballObj.confidence < confidenceValve)
            continue;

        // 防止把天上的灯识别为球
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 10.0)
            continue;

        // TODO 加入更多排除参数, 例如在身体上, 明显在球场外, 位置突然大幅度变化等
        // 被遮挡的条件要加入. 如果突然消失, 没有遮挡的话, 则只相信一小会儿, 如果有遮挡, 可以相信比较长的时间.

        // 找出剩下的球中, 置信度最高的
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    if (indexRealBall >= 0)
    { // 检测到球了
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];

        tree->setEntry<bool>("ball_location_known", true);
    }
    else
    { // 没有检测到球
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;
        data->ball.confidence = 0;
    }

    // 计算机器人到球的向量, 在 field 坐标系中的方向
    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
}

void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    const double confidenceValve = 0.1; // confidence 低于这个阈值, 排除

    data->markings.clear(); // 清空 brain 中的记忆

    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        // 判断: 如果置信度太低, 则认为是误检
        if (marking.confidence < confidenceValve)
            continue;

        // 排除天的上误识别标记
        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)
            continue;

        // 如果通过了重重考验, 则记入 brain
        data->markings.push_back(marking);
    }
}