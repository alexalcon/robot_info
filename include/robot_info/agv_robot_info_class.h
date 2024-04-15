#pragma once

#include "robot_info/robot_info_class.h"
#include "robot_info/hydraulic_system_monitor.h"

class AGVRobotInfo : public RobotInfo {
// data members specific to agv robots
private:
    std::string maximum_payload;
    HydraulicSystemMonitor hydraulic_system;

// constructors and virtual function
public:
    // inherits constructors from RobotInfo
    using RobotInfo::RobotInfo;

    /**
     * @brief User-defined default constructor for AGVRobotInfo.
     * @param nh Pointer to a ROS node handle.
    */
    AGVRobotInfo(ros::NodeHandle* nh);

    /**
     * @brief Delegated Parameterized constructor for AGVRobotInfo that sets maximum payload.
     * @param nh ROS node handle.
     * @param robot_description User-defined robot description attribute.
     * @param serial_number User-defined robot serial number attribute.
     * @param ip_address User-defined robot ip address attribute.
     * @param firmware_version User-defined robot firmware version attribute.
     * @param maximum_payload User-defined maximum payload capacity of the AGV.
     */
    AGVRobotInfo(ros::NodeHandle* nh, 
                 const std::string& robot_description,
                 const std::string& serial_number,
                 const std::string& ip_address,
                 const std::string& firmware_version,
                 const std::string& maximum_payload);

    /**
     * @brief Publishes AGV-specific data to ROS network.
     */
    void publish_data() override;
};
