#include "robot_info/agv_robot_info_class.h"

// constructors
//--------------------------------------------------------------------------------------------
// user-defined default constructor
AGVRobotInfo::AGVRobotInfo(ros::NodeHandle* nh) : RobotInfo(nh) {
    this->maximum_payload = "maximum_payload: 100 Kg";
}

// delegated parameterized constructor
AGVRobotInfo::AGVRobotInfo(ros::NodeHandle* nh, const std::string& maximum_payload)
    : RobotInfo(nh), // calls the RobotInfo constructor with the node handle
      maximum_payload(maximum_payload) {// sets the maximum payload 
    // all initialization is done by the base class constructor and member initializer list.
}
//--------------------------------------------------------------------------------------------

// publisher set up
//--------------------------------------------------------------------------
// override the publish_data virtual function to include AGV-specific data
void AGVRobotInfo::publish_data() {
    robotinfo_msgs::RobotInfo10Fields msg;
    
    // use the base class method to populate common fields
    this->populateMessage(msg);

    // add or override with AGV-specific data
    msg.data_field_05 = this->maximum_payload;

    // ROS_INFO("Overriden publish_data() function called.");
    robot_info_publisher.publish(msg);
}
//--------------------------------------------------------------------------
