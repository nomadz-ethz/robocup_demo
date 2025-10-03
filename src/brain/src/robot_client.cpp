#include <cmath>
#include "brain.h"
#include "robot_client.h"
#include "booster_interface/message_utils.hpp"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

void RobotClient::init()
{
    publisher = brain->create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopicReq", 10);
}

int RobotClient::call(booster_interface::msg::BoosterApiReqMsg msg)
{
    std::string uuid = gen_uuid();
    auto message = booster_msgs::msg::RpcReqMsg();
    message.uuid = uuid;

    nlohmann::json req_header;
    req_header["api_id"] = msg.api_id;
    message.header = req_header.dump();
    message.body = msg.body;
    publisher->publish(message);
    return 0;
}

int RobotClient::moveHead(double pitch, double yaw)
{
    // 软限位
    yaw = cap(yaw, brain->config->headYawLimitLeft, brain->config->headYawLimitRight);
    pitch = max(pitch, brain->config->headPitchLimitUp);

    brain->log->setTimeNow();
    auto level = (fabs(pitch > 2.0) || fabs(yaw > 2.0)) ? rerun::TextLogLevel::Error : rerun::TextLogLevel::Info;
    brain->log->log("debug/move_head", rerun::TextLog(format("pitch: %.1f, yaw: %.1f", pitch, yaw)).with_level(level));

    return call(booster_interface::CreateRotateHeadMsg(pitch, yaw));
}

int RobotClient::standUp()
{
    booster_interface::msg::BoosterApiReqMsg msg;
    msg.api_id = static_cast<int64_t>(booster::robot::b1::LocoApiId::kGetUp);
    nlohmann::json body;
    msg.body = body.dump();

    return call(msg);
}


int RobotClient::enterDamping()
{
    return call(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kDamping));
}

int RobotClient::waveHand(bool doWaveHand)
{
    return call(booster_interface::CreateWaveHandMsg(booster::robot::b1::HandIndex::kRightHand, doWaveHand ? booster::robot::b1::HandAction::kHandOpen : booster::robot::b1::HandAction::kHandClose));
}

int RobotClient::setVelocity(double x, double y, double theta, bool applyMinX, bool applyMinY, bool applyMinTheta)
{
    brain->log->setTimeNow();
    brain->log->log("RobotClient/setVelocity_in",
                    rerun::TextLog(format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta)));

    // 速度指令太小时, 给一个最小速度, 以防止不响应 TODO 转为参数化
    double minx = 0.05, miny = 0.08, mintheta = 0.05;
    if (applyMinX && fabs(x) < minx && fabs(x) > 1e-5)
        x = x > 0 ? minx : -minx;
    if (applyMinY && fabs(y) < miny && fabs(y) > 1e-5)
        y = y > 0 ? miny : -miny;
    if (applyMinTheta && fabs(theta) < mintheta && fabs(theta) > 1e-5)
        theta = theta > 0 ? mintheta : -mintheta;
    x = cap(x, brain->config->vxLimit, -brain->config->vxLimit);
    y = cap(y, brain->config->vyLimit, -brain->config->vyLimit);
    theta = cap(theta, brain->config->vthetaLimit, -brain->config->vthetaLimit);
    
    // log simulated path based on velocity
    vector<Pose2D> path = {{0, 0, 0}}; // 坐标系是以机器人的位置为 0,0, linear 速度方向为 theta = 0 的坐标系
    double v = norm(x, y);
    double simStep = 1e-1; double simLength = 5.;
    for (int i = 0; i < simLength/simStep; i++) {
        double dt = simStep * (i + 1);
        if (fabs(theta) < 1e-3) path.push_back({v * dt, 0, 0});
        else path.push_back({v/theta * sin(theta * dt), v/theta * (1 - cos(theta * dt)), theta * dt});
    }
    vector<Pose2D> path_f = {};
    for (int i = 0; i < path.size(); i++) {
        auto p = trans(
            path[i].x, path[i].y, path[i].theta, 
            brain->data->robotPoseToField.x, 
            brain->data->robotPoseToField.y, 
            brain->data->robotPoseToField.theta + atan2(y, x), 
            "back"
        );
        path_f.push_back({p[0], -p[1], -p[2]});
    }
    
    vector<rerun::components::Vector2D> vectors = {};
    vector<rerun::components::Position2D> origins = {};
    vector<rerun::components::Color> colors;
    for (int i = 1; i < path_f.size(); i++) {
        auto p0 = path_f[i-1];
        auto p1 = path_f[i];
        origins.push_back({p0.x, p0.y});
        vectors.push_back({p1.x - p0.x, p1.y - p0.y});
        colors.push_back({0, 0, 255 * (1 - i / path_f.size() / 2.0)});
    }

    if (path_f.size() > 0) {
        auto p = path_f[0];
        brain->log->log(
            "field/velocity",
            rerun::Arrows2D::from_vectors({{v * cos(p.theta), v * sin(p.theta)}})
                .with_origins({{p.x, p.y}})
                .with_colors(0xCCCCCCFF)
                .with_radii(0.005)
                .with_draw_order(40)
        );
    }
    _vx = x; _vy = y; _vtheta = theta; // remember last command. 可以近似作为当前机器人的速度使用. 
    _lastCmdTime = brain->get_clock()->now();
    if (fabs(_vx) > 1e-3 || fabs(_vy) > 1e-3 || fabs(_vtheta) > 1e-3) _lastNonZeroCmdTime = brain->get_clock()->now();
    brain->log->log("RobotClient/setVelocity_out",
        rerun::TextLog(format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta)));
    return call(booster_interface::CreateMoveMsg(x, y, theta));
}

int RobotClient::crabWalk(double angle, double speed) {
    double vxFactor = brain->config->vxFactor;   // 用于调整 vx, vx *= vxFactor, 以补偿 x, y 方向的速度参数与实际速度比例的偏差, 使运动方向准确
    double yawOffset = brain->config->yawOffset; // 用于补偿定位角度的偏差
    double vxLimit = brain->config->vxLimit;
    double vyLimit = brain->config->vyLimit;

    // 计算速度指令
    double cmdAngle = angle + yawOffset;
    double vx = cos(cmdAngle) * speed * vxFactor;
    double vy = sin(cmdAngle) * speed;

    if (fabs(vy) > vyLimit) {
        vx *= vyLimit / fabs(vy);
        vy = vyLimit * fabs(vy) / vy;
    }

    return setVelocity(vx, vy, 0);
}



bool RobotClient::isStandingStill(double timeBuffer) {
    return fabs(_vx) < 1e-3 && fabs(_vy) < 1e-3 && fabs(_vtheta) < 1e-3 && brain->msecsSince(_lastNonZeroCmdTime) > timeBuffer;
}

int RobotClient::moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    Pose2D target_f, target_r; // 移动目标在 field 和 robot 坐标系中的 Pose
    static Pose2D target_temp_r; // 用于避障的 temp_target
    static rclcpp::Time timeEndTempTarget = brain->get_clock()->now(); // 在此时间之前, 执行 temp_target, 而不是真正的 target

    if (brain->get_clock()->now() < timeEndTempTarget) { // 执行用于避障的暂时目标
        target_r = target_temp_r;
        vxLimit = 0.4;
        vyLimit = 0.4;
    } else { // 避障时间已过, 执行真正的目标
        target_f.x = tx;
        target_f.y = ty;
        target_f.theta = ttheta;
        target_r = brain->data->field2robot(target_f);
    }

    double targetAngle = atan2(target_r.y, target_r.x);
    double targetDist = norm(target_r.x, target_r.y);

    double vx, vy, vtheta;

    // 已经到达目标?
    if (
        (fabs(brain->data->robotPoseToField.x - target_f.x) < xTolerance) && (fabs(brain->data->robotPoseToField.y - target_f.y) < yTolerance) && (fabs(toPInPI(brain->data->robotPoseToField.theta - target_f.theta)) < thetaTolerance))
    {
        return setVelocity(0, 0, 0);
    }

    // 较远时
    static double breakOscillate = 0.0;
    if (targetDist > longRangeThreshold - breakOscillate)
    {
        breakOscillate = 0.5;

        // 角度较大, 先转向目标点
        if (fabs(targetAngle) > turnThreshold)
        {
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
        } else {
            vx = cap(target_r.x, vxLimit, -vxLimit);
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);

        }
    } else { // 比较近了
        breakOscillate = 0.0;
        vx = cap(target_r.x, vxLimit, -vxLimit);
        vy = cap(target_r.y, vyLimit, -vyLimit);
        vtheta = cap(target_r.theta, vthetaLimit, -vthetaLimit);
    }

    // 避障
    if (avoidObstacle) {
        double etc = msecsToCollide(vx, vy, vtheta); // estimated time to collide
        double eta = norm(target_r.x, target_r.y) / max(1e-5, norm(vx, vy)) * 1000;// estimated time to arrive

        auto color = 0x00FF00FF;
        if (etc < 3000) color = 0xFF0000FF;
        else if (etc < 5000) color = 0x00FF00FF;
        else color = 0x00FF00FF;

        brain->log->setTimeNow();
        brain->log->log(
            "tick/time_to_collide",
            rerun::LineStrips2D(rerun::LineStrip2D({{10., -200.}, {10. + min(etc / 10, 1260.), -200.}}))
                .with_colors(color)
                .with_radii(2.0)
                .with_draw_order(10)
                .with_labels({format("etc:  %.0fms", etc)})
        );
        if (etc < min(eta, 3000.)) {
            std::srand(std::time(0));
            vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01; // step on spot, don't full stop
            vy = 0.0;
            vtheta = 0;

            auto rbtPose = brain->data->robotPoseToField;
            double theta0 = toPInPI(atan2(ty - rbtPose.y, tx - rbtPose.x) - rbtPose.theta);
            double theta1;
            double vx_temp, vy_temp;
            for (int i = 0; i < 9; i++) {
                theta1 = theta0 + M_PI / 9 * i;
                vx_temp = 0.4 * cos(theta1);
                vy_temp = 0.4 * sin(theta1);
                if (msecsToCollide(vx_temp, vy_temp, 0) > 3000) break;

                theta1 = theta0 - M_PI / 9 * i;
                vx_temp = 0.4 * cos(theta1);
                vy_temp = 0.4 * sin(theta1);
                if (msecsToCollide(vx_temp, vy_temp, 0) > 3000) break;
            }
            brain->log->log("debug/avoidObstacle", rerun::TextLog(format(
                "theta1 = %.2f",
                theta1
            )));
            target_temp_r.x = 1 * cos(theta1);
            target_temp_r.y = 1 * sin(theta1);
            timeEndTempTarget = brain->get_clock()->now() + rclcpp::Duration(3.5, 0.0);
        }
        else if (etc < min(eta, 6000.)) {
            vtheta += 0.2;
        }
    }

    return setVelocity(vx, vy, vtheta);
}

int RobotClient::moveToPoseOnField2(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    static string mode = "longRange";
    static bool isBacking = false;
    static rclcpp::Time timeEndAvoid = brain->get_clock()->now();
    static double avoidDir = 1.0; // 1.0 for turn left, -1.0 for turn right
    const double SAFE_DIST = brain->config->safeDistance;
    const double AVOID_SECS = brain->config->avoidSecs;

    

    double range = norm(tx - brain->data->robotPoseToField.x, ty - brain->data->robotPoseToField.y);
    if (range > longRangeThreshold * (mode == "longRange" ? 0.9 : 1.0)) {
        mode = "longRange";
    } else {
        mode = "shortRange";
    }

    double vx, vy, vtheta;
    double tarDir = atan2(ty - brain->data->robotPoseToField.y, tx - brain->data->robotPoseToField.x);
    double faceDir = brain->data->robotPoseToField.theta;
    double tarDir_r = toPInPI(tarDir - faceDir); 
    auto now = brain->get_clock()->now();
    if (mode == "longRange") { 

        if (fabs(tarDir_r) > turnThreshold) { 
            vx = 0.0;
            vy = 0.0;
            vtheta = tarDir_r;
        } else { 
            vx = cap(range, vxLimit, -vxLimit);
            vy = 0.0;
            vtheta = tarDir_r;
        }
       
        if (avoidObstacle) {
            if (now < timeEndAvoid) { 
                double distToObstacle = brain->distToObstacle(0);
                if (distToObstacle < SAFE_DIST / 2.0) { 
                    isBacking = true;
                    timeEndAvoid = now + rclcpp::Duration(AVOID_SECS, 0.0);
                    avoidDir = brain->calcAvoidDir(tarDir_r, SAFE_DIST) > 0 ? 1.0 : -1.0;
                    vx = -0.2;
                    vy = avoidDir * 0.2;
                    vtheta = 0.0; 
                }else if (brain->distToObstacle(0) < SAFE_DIST + (isBacking ? 0.5 : 0.0)) { 
                    isBacking = false;
                    timeEndAvoid = now + rclcpp::Duration(AVOID_SECS, 0.0);
                    avoidDir = brain->calcAvoidDir(tarDir_r, SAFE_DIST) > 0 ? 1.0 : -1.0;
                    vx = 0.0;
                    vy = 0.0;
                    vtheta = avoidDir * brain->config->vthetaLimit;
                } else {
                    vx = vxLimit;
                    if (brain->distToObstacle(tarDir_r) < SAFE_DIST * 2) vxLimit *= 0.5;
                    vy = 0.0;
                    vtheta = 0.0;
                }
            } else {
                double distToObstacle = brain->distToObstacle(tarDir_r);
                if (distToObstacle < SAFE_DIST * 2) vxLimit = 0.5;
                if (distToObstacle < SAFE_DIST) {
                    timeEndAvoid = now + rclcpp::Duration(AVOID_SECS, 0.0);
                    avoidDir = brain->calcAvoidDir(tarDir_r, SAFE_DIST) > 0 ? 1.0 : -1.0;

                    vx = 0.0;
                    vy = 0.0;
                    vtheta = 0.0;
                }
            }

            // double distToObstacle = brain->distToObstacle(tarDir_r);
            // double safeAngle = tarDir_r;
            // if (distToObstacle < SAFE_DIST) {
            //     cout << "distToObstacle < SAFE_DIST" << endl;
            //     // find a safe and close direction
            //     auto res = brain->findSafeDirections(tarDir_r, SAFE_DIST);
            //     bool leftFound = res[0] > 0.5;
            //     bool rightFound = res[2] > 0.5;
            //     double angleLeft = res[1];
            //     double angleRight = res[3];
            //     if (leftFound && rightFound) {
            //         if (fabs(angleLeft - faceDir) < fabs(angleRight - faceDir)) {
            //             safeAngle = angleLeft;
            //         } else {
            //             safeAngle = angleRight;
            //         }
            //     } else if (leftFound) {
            //         safeAngle = angleLeft;
            //     } else if (rightFound) {
            //         safeAngle = angleRight;
            //     }

            //     if (fabs(toPInPI(faceDir - tarDir_r)) - fabs(toPInPI(safeAngle - tarDir_r)) > 0) {
            //         vx = 0.5;
            //         vy = 0.0;
            //         vtheta = toPInPI(safeAngle - faceDir);
            //     } else {
            //         vx = 0.0;
            //         vy = 0.0;
            //         vtheta = toPInPI(safeAngle - faceDir);
            //     }
            // } else {

            // }
        }
    } else if (mode == "shortRange") { 
        

        vx = range * cos(tarDir_r);
        vy = range * sin(tarDir_r);
        vtheta = toPInPI(ttheta - faceDir); 
        if (fabs(vx) < xTolerance && fabs(vy) < yTolerance && fabs(vtheta) < thetaTolerance) {
            vx = 0.0;
            vy = 0.0;
            vtheta = 0.0;
        }


        if (avoidObstacle) { 
            double distToObstacle = brain->distToObstacle(tarDir_r);

            if (distToObstacle < SAFE_DIST) {
                vx = 0.0;
                vy = 0.0;
                vtheta = tarDir_r;
            }
        }
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit); 
    return setVelocity(vx, vy, vtheta);
}

int RobotClient::moveToPoseOnField3(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    static string mode = "longRange";
    static bool should_avoid_in_move = false;
    bool isFreekickStartPlacing = brain->isFreekickStartPlacing();

    const double SAFE_DIST = isFreekickStartPlacing ? brain->get_parameter("obstacle_avoidance.freekick_start_placing_safe_distance").get_value<double>() : brain->config->safeDistance;
    const double AVOID_SECS = isFreekickStartPlacing ? brain->get_parameter("obstacle_avoidance.freekick_start_placing_avoid_secs").get_value<double>() : brain->config->avoidSecs;

    double range = norm(tx - brain->data->robotPoseToField.x, ty - brain->data->robotPoseToField.y);
    if (range > longRangeThreshold * (mode == "longRange" ? 0.9 : 1.0)) { 
        mode = "longRange";
    } else {
        mode = "shortRange";
    }

    double vx, vy, vtheta;
    double tarDir = atan2(ty - brain->data->robotPoseToField.y, tx - brain->data->robotPoseToField.x);
    double faceDir = brain->data->robotPoseToField.theta;
    double tarDir_r = toPInPI(tarDir - faceDir); 
    if (mode == "longRange") { 
        // brain->log->log("debug/freekick_position", rerun::TextLog(format(
        //         "long range"
        // )));
        if (fabs(tarDir_r) > turnThreshold) { 
            vx = 0.0;
            vy = 0.0;
            vtheta = tarDir_r;
        } else { 
            vx = cap(range, vxLimit, -vxLimit);
            vy = 0.0;
            vtheta = tarDir_r;
        }
       
        if (avoidObstacle) {
            double mostViableDir = 0; 
            double distToObstacleMostViable = brain->distToObstacle(0); 
            for (size_t i = 1; i <= 10; i++)
            {
                double tarDir_r_temp = toPInPI(0 + M_PI / 10 * i);
                double distToObstacle_temp = brain->distToObstacle(tarDir_r_temp);
                if (distToObstacle_temp > SAFE_DIST) {
                    mostViableDir = tarDir_r_temp;
                    break;
                }

                tarDir_r_temp = toPInPI(0 - M_PI / 10 * i);
                distToObstacle_temp = brain->distToObstacle(tarDir_r_temp);
                if (distToObstacle_temp > SAFE_DIST) {
                    mostViableDir = tarDir_r_temp;
                    break;
                }
            }
            if (should_avoid_in_move) {
                double distToObstacle = brain->distToObstacle(0);
                // brain->log->log("debug/freekick_position", rerun::TextLog(format(
                //         "side move distToObstacle: %.2f, SAFE_DIST: %.2f",
                //         distToObstacle, SAFE_DIST
                // )));
                vx = 0;
                vy = (mostViableDir - tarDir_r > 0 ? 0.3 : -0.3); 
                vtheta = 0.0;
                if (distToObstacle < SAFE_DIST) {
                    should_avoid_in_move = true;
                } else {
                    vx = vxLimit;
                    if (brain->distToObstacle(tarDir_r) < SAFE_DIST * 2) vxLimit *= 0.5;
                    vy = 0.0;
                    vtheta = 0.0;
                    should_avoid_in_move = false; 
                }
            } else {
                double distToObstacle = brain->distToObstacle(tarDir_r);
                if (distToObstacle < SAFE_DIST * 3) vxLimit = 0.5;
                if (distToObstacle < SAFE_DIST) {
                    should_avoid_in_move = true;

                    // brain->log->log("debug/freekick_position", rerun::TextLog(format(
                    //         "long range zero case"
                    // )));

                    vx = 0.0;
                    vy = 0.0;
                    vtheta = 0.0;
                }
            }

        }
    } else if (mode == "shortRange") { 
        // brain->log->log("debug/freekick_position", rerun::TextLog(format(
        //     "shortRange"
        // )));

        vx = range * cos(tarDir_r);
        vy = range * sin(tarDir_r);
        vtheta = toPInPI(ttheta - faceDir); 
        if (fabs(vx) < xTolerance && fabs(vy) < yTolerance && fabs(vtheta) < thetaTolerance) {
            vx = 0.0;
            vy = 0.0;
            vtheta = 0.0;

            // brain->log->log("debug/freekick_position", rerun::TextLog(format(
            //     "shortRange zero case"
            // )));
        }


        if (avoidObstacle) { 
            double distToObstacle = brain->distToObstacle(tarDir_r);

            double mostViableDir = tarDir_r; 
            double distToObstacleMostViable = distToObstacle;
            for (size_t i = 1; i <= 10; i++)
            {
                double tarDir_r_temp = toPInPI(tarDir_r + M_PI / 10 * i);
                double distToObstacle_temp = brain->distToObstacle(tarDir_r_temp);
                if (distToObstacle_temp > SAFE_DIST) {
                    mostViableDir = tarDir_r_temp;
                    break;
                }

                tarDir_r_temp = toPInPI(tarDir_r - M_PI / 10 * i);
                distToObstacle_temp = brain->distToObstacle(tarDir_r_temp);
                if (distToObstacle_temp > SAFE_DIST) {
                    mostViableDir = tarDir_r_temp;
                    break;
                }
            }

            if (distToObstacle < SAFE_DIST) {
                // brain->log->log("debug/freekick_position", rerun::TextLog(format(
                //     "shot range: %.2f, SAFE_DIST: %.2f",
                //     distToObstacle, SAFE_DIST
                // )));
                vx = 0.0;
                vy = mostViableDir == tarDir_r ? 0.0 : (mostViableDir - tarDir_r > 0 ? 0.2 : -0.2); 
                vtheta = tarDir_r; 
            }
        }
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit); 
    return setVelocity(vx, vy, vtheta);
}

double RobotClient::msecsToCollide(double vx, double vy, double vtheta, double maxTime) {
    auto robots = brain->data->getRobots();

    if (robots.size() == 0) return maxTime; 
    if (vx < 1e-3 && vy < 1e-3) return maxTime; 

    brain->log->setTimeNow();

    // 先简化计算, 不考虑 vtheta
    double x0 = brain->data->robotPoseToField.x;
    double y0 = brain->data->robotPoseToField.y;
    double robotTheta = brain->data->robotPoseToField.theta;

    Line path = {
        x0, y0,
        x0 + vx * cos(robotTheta) * maxTime / 1000, y0 + vy * sin(robotTheta) * maxTime / 1000
    };

    double minDist = 1e6;
    const double SAFE_DIST = 0.6;
    for (int i = 0; i < robots.size(); i++) {
        auto pose = robots[i].posToField;
        Point2D p = {pose.x, pose.y};
        double distToPath = pointMinDistToLine(p, path);
        brain->log->log("debug/msecsToCollide", rerun::TextLog(format(
            "i = %d, rbt at (%.2f, %.2f), obstacle at (%.2f, %.2f), distToPath = %.2f",
            i, x0, y0, p.x, p.y, distToPath
        )));
        if (distToPath < SAFE_DIST) {
            Line line = {x0, y0, pose.x, pose.y};
            double angle = angleBetweenLines(line, path);
            double l = norm(pose.x - x0, pose.y - y0);
            double d = l*cos(angle) - sqrt(SAFE_DIST*SAFE_DIST - l*l*sin(angle)*sin(angle));
            brain->log->log("debug/msecsToCollide", rerun::TextLog(format(
                "angle: %.2f, l: %.2f, d: %.2f",
                angle, l, d
            )));
            if (d < minDist) minDist = d;
        }
    }

    return min(maxTime, minDist / norm(vx, vy) * 1000);
}
