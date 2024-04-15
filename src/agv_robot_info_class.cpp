#include "robot_info/agv_robot_info_class.h"

// constructors
//--------------------------------------------------------------------------------------------
// user-defined default constructor
AGVRobotInfo::AGVRobotInfo(ros::NodeHandle* nh) : RobotInfo(nh) {
    this->maximum_payload = "maximum_payload: 100 Kg";
}

// delegated parameterized constructor
AGVRobotInfo::AGVRobotInfo(ros::NodeHandle* nh, 
                           const std::string& robot_description,
                           const std::string& serial_number,
                           const std::string& ip_address,
                           const std::string& firmware_version,
                           const std::string& maximum_payload)
    : RobotInfo(nh, // calls the RobotInfo constructor with the node handle    
                robot_description,
                serial_number,
                ip_address,
                firmware_version), 
      maximum_payload(maximum_payload) { // sets the maximum payload 
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

    // set hydraulic system physical technical parameters
    hydraulic_system.setOilTemperature("hydraulic_oil_temperature: 45C");
    hydraulic_system.setOilTankFillLevel("hydraulic_oil_tank_fill_level: 100 %");
    hydraulic_system.setOilPressure("hydraulic_oil_pressure: 250 [bar]");

    // populate the message fields for the hydraulic system
    msg.data_field_06 = hydraulic_system.getOilTemperature();
    msg.data_field_07 = hydraulic_system.getOilTankFillLevel();
    msg.data_field_08 = hydraulic_system.getOilPressure();

    // ROS_INFO("Overriden publish_data() function called.");
    robot_info_publisher.publish(msg);
}
//--------------------------------------------------------------------------
