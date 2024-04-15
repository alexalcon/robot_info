#include "robot_info/agv_robot_info_class.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "agv_robot_info_node");
    ros::NodeHandle nh;

    AGVRobotInfo agv_robot_info = AGVRobotInfo(&nh);
    // RobotInfo robot_info = RobotInfo(&nh, "A", "B", "C", "D");
    
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        agv_robot_info.publish_data();

        ros::spinOnce();

        loop_rate.sleep();
    }
   
    return 0;
} // end function main