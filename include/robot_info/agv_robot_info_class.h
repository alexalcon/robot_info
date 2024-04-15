#pragma once

#include "robot_info/robot_info_class.h"

class AGVRobotInfo : public RobotInfo {
// data members specific to agv robots
private:
    std::string maximum_payload;

// constructors and virtual function
public:
    // inherits constructors from RobotInfo
    using RobotInfo::RobotInfo;
    
    AGVRobotInfo(ros::NodeHandle* nh);

    /**
     * @brief Parameterized constructor for AGVRobotInfo that sets maximum payload.
     * @param nh ROS node handle.
     * @param maximum_payload Maximum payload capacity of the AGV.
     */
    AGVRobotInfo(ros::NodeHandle* nh, const std::string& maximum_payload);

    /**
     * @brief Publishes AGV-specific data to ROS network.
     */
    void publish_data() override;
};
