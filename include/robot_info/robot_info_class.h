#pragma once

#include <ros/ros.h>
#include <string>
#include "robotinfo_msgs/RobotInfo10Fields.h"

/**
 * @class RobotInfo
 * @brief Base class for robot information publishing.
 *
 * Encapsulates general robot data for ROS publishing. Includes virtual method 
 * publish_data() for broadcasting robot's description, serial number, IP address, 
 * and firmware version. Designed to be extended by subclasses like AGVRobotInfo
 * to add specific information such as maximum payload.
 */
class RobotInfo {
// data members
private:
    // robot technical specifications
    std::string robot_description;
    std::string serial_number;
    std::string ip_address;
    std::string firmware_version;

    // publishing topic
    std::string robot_info_topic;  

// inherited data and functions
protected:
    // pointer to a ROS node handle
    ros::NodeHandle* nh; 
    
    // custom message data
    // robotinfo_msgs::RobotInfo10Fields msg;

    // ROS publisher object
    ros::Publisher robot_info_publisher;
    
    /**
     * @brief Initializes the robot information publisher.
     *
     * This function advertises the topic for robot information with a predefined
     * message queue size. It is meant to be called during construction to set up
     * the publisher.
     */
    void initRobotInfoPublisher();

    /**
     * @brief Populates a RobotInfo message with data from this class.
     * @param msg The message object to populate.
     */
    void populateMessage(robotinfo_msgs::RobotInfo10Fields& msg) const;

// constructors and virtual function
public:
    /**
     * @brief User-defined default constructor for RobotInfo.
     * @param nh Pointer to a ROS node handle.
    */
    RobotInfo(ros::NodeHandle* nh);
    
    /**
     * @brief Delegated parameterized constructor for RobotInfo.
     * @param robot_description User-defined robot description attribute.
     * @param serial_number User-defined robot serial number attribute.
     * @param ip_address User-defined robot ip address attribute.
     * @param firmware_version User-defined robot firmware version attribute.
    */
    RobotInfo(ros::NodeHandle* nh,
              const std::string& robot_description,
              const std::string& serial_number,
              const std::string& ip_address,
              const std::string& firmware_version);

    /**
     * @brief Virtual method for data publishing over ROS.
     *
     * Designed for override in derived classes to publish extended robot data. The base
     * implementation publishes essential robot details. Extend in AGVRobotInfo to include
     * AGV-specific data like maximum payload.
     *
     * The data is published using a custom ROS message type, and the method is expected to
     * be called periodically to send the latest information to any subscribers listening
     * on the robot_info topic.
    */
    virtual void publish_data();
}; // end class RobotInfo