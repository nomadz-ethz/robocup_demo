#pragma once

#include <string>
#include <mutex>
#include <tuple>

#include <sensor_msgs/msg/image.hpp>
#include "booster_interface/msg/odometer.hpp"
#include <Eigen/Dense> 

#include "types.h"
#include "RoboCupGameControlData.h"

using namespace std;

/**
 * BrainData 类，记录 Brain 在决策中需要用到的所在数据，区分于 BrainConfig，这里是运行时数据（动态）
 * 针对数据处理的一些工具函数，也可以放到这里来
 */
class BrainData
{
public:
    BrainData();
    /* ------------------------------------ 球赛相关状态量 ------------------------------------ */

    int score = 0;
    int oppoScore = 0;
    int penalty[HL_MAX_NUM_PLAYERS]; 
    int oppoPenalty[HL_MAX_NUM_PLAYERS]; 
    bool isKickingOff = false; 
    rclcpp::Time kickoffStartTime; 
    bool isFreekickKickingOff = false; 
    rclcpp::Time freekickKickoffStartTime; 
    int liveCount = 0; 
    int oppoLiveCount = 0; 
    string realGameSubState; 

    /* ------------------------------------ 数据记录 ------------------------------------ */
    
   
    Pose2D robotPoseToOdom;  
    Pose2D odomToField;      
    Pose2D robotPoseToField; 

    double headPitch; 
    double headYaw;  
    Eigen::Matrix4d camToRobot = Eigen::Matrix4d::Identity(); 


    bool ballDetected = false;   
    GameObject ball;              
    GameObject tmBall;           
    double robotBallAngleToField; 


    inline vector<GameObject> getRobots() const {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        return _robots;
    }
    inline void setRobots(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        _robots = newVec;
    }


    inline vector<GameObject> getGoalposts() const {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        return _goalposts;
    }
    inline void setGoalposts(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        _goalposts = newVec;
    }


    inline vector<GameObject> getMarkings() const {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        return _markings;
    }
    inline void setMarkings(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        _markings = newVec;
    }

    inline vector<FieldLine> getFieldLines() const {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        return _fieldLines;
    }
    inline void setFieldLines(const vector<FieldLine>& newVec) {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        _fieldLines = newVec;
    }


    inline vector<GameObject> getObstacles() const {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        return _obstacles;
    }
    inline void setObstacles(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        _obstacles = newVec;
    }


    double kickDir = 0.; 
    string kickType = "shoot"; 
    bool isDirectShoot = false; 


    TMStatus tmStatus[HL_MAX_NUM_PLAYERS]; 
    int tmCmdId = 0; 
    rclcpp::Time tmLastCmdChangeTime; 
    int tmMyCmd = 0; 
    int tmMyCmdId = 0; 
    int tmReceivedCmd = 0; 
    bool tmImLead = true; 
    bool tmImAlive = true; 
    double tmMyCost = 0.;


    int discoveryMsgId = 0;
    rclcpp::Time discoveryMsgTime;
    int sendId = 0;
    rclcpp::Time sendTime;
    int receiveId[HL_MAX_NUM_PLAYERS];
    rclcpp::Time receiveTime[HL_MAX_NUM_PLAYERS]; 
    string tmIP;
    

    RobotRecoveryState recoveryState = RobotRecoveryState::IS_READY;
    bool isRecoveryAvailable = false; 
    int currentRobotModeIndex = -1;
    int recoveryPerformedRetryCount = 0; 
    bool recoveryPerformed = false;


    rclcpp::Time timeLastDet; 
    bool camConnected = false; 
    rclcpp::Time timeLastLineDet; 
    rclcpp::Time lastSuccessfulLocalizeTime;
    rclcpp::Time timeLastGamecontrolMsg; 
    rclcpp::Time timeLastLogSave; 
    VisionBox visionBox;  
    rclcpp::Time lastTick; 


    /**
     * @brief 按类型获取 markings
     * 
     * @param type set<string>, 空 set 代表所有类型, 否则代表指定的类型 "LCross" | "TCross" | "XCross" | "PenaltyPoint"
     * 
     * @return vector<GameObject> 类型符合的 markings
     */
    vector<GameObject> getMarkingsByType(set<string> types={});


    vector<FieldMarker> getMarkersForLocator();


    Pose2D robot2field(const Pose2D &poseToRobot);


    Pose2D field2robot(const Pose2D &poseToField);

private:
    vector<GameObject> _robots = {}; 
    mutable std::mutex _robotsMutex;

    vector<GameObject> _goalposts = {}; 
    mutable std::mutex _goalpostsMutex;

    vector<GameObject> _markings = {};                             
    mutable std::mutex _markingsMutex;

    vector<FieldLine> _fieldLines = {};
    mutable std::mutex _fieldLinesMutex;

    vector<GameObject> _obstacles = {};
    mutable std::mutex _obstaclesMutex;

};