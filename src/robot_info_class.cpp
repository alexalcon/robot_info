#include "robot_info/robot_info_class.h"

// constructors
//-----------------------------------------------------------------------------------
// user-defined default constructor
RobotInfo::RobotInfo(ros::NodeHandle *nh) {
  // ROS node handle
  this->nh = nh;

  // topic name to publish robot info
  this->robot_info_topic = "robot_info";

  // default robot technical specifications info
  this->robot_description = "robot_description: Mir100";
  this->serial_number = "serial_number: 567A359";
  this->ip_address = "ip_address: 169.254.5.180";
  this->firmware_version = "firmware_version: 3.5.8";

  // custom message data (robot info) initialization
  // msg.data_field_01 = this->robot_description;
  // msg.data_field_02 = this->serial_number;
  // msg.data_field_03 = this->ip_address;
  // msg.data_field_04 = this->firmware_version;

  // function to initialize the publisher
  initRobotInfoPublisher();
} // end user-defined default constructor

// delegated parameterized constructor
RobotInfo::RobotInfo(ros::NodeHandle *nh, 
                     const std::string &robot_description,
                     const std::string &serial_number,
                     const std::string &ip_address,
                     const std::string &firmware_version)
    : RobotInfo::RobotInfo(nh) { // this calls the user-defined default constructor

  /**
   * At this point, the default constructor has finished.
   * The nh, robot_info_topic are already set to "robot info" 
   * by the user-defined default constructor.
   */

  // user-defined robot technical specifications info
  this->robot_description = robot_description;
  this->serial_number = serial_number;
  this->ip_address = ip_address;
  this->firmware_version = firmware_version;

  /**
   * There is no need to call initRobotInfoPublisher() again,
   * because it was already called in the default constructor.
   */
} // end delegated parameterized constructor
//-----------------------------------------------------------------------------------

// populate the message fields with data from the RobotInfo object
void RobotInfo::populateMessage(robotinfo_msgs::RobotInfo10Fields &msg) const {
  msg.data_field_01 = this->robot_description;
  msg.data_field_02 = this->serial_number;
  msg.data_field_03 = this->ip_address;
  msg.data_field_04 = this->firmware_version;
}

// publisher set up
//-----------------------------------------------------------------------------------------------
// function to initialize robot info publisher
void RobotInfo::initRobotInfoPublisher() {
  this->robot_info_publisher = nh->advertise<robotinfo_msgs::RobotInfo10Fields>(
      this->robot_info_topic, 1000);
  ROS_INFO("Robot info publisher initialized.");
} // end function initRobotInfoPublisher

// custom message data (robot info) publisher virtual function
void RobotInfo::publish_data() {
  robotinfo_msgs::RobotInfo10Fields msg;

  // use the base class method to populate common fields
  this->populateMessage(msg);

  robot_info_publisher.publish(msg);
} // end function publish_data
//-----------------------------------------------------------------------------------------------
