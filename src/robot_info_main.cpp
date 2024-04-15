#include "robot_info/robot_info_class.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_info_node");
    ros::NodeHandle nh;

    RobotInfo robot_info = RobotInfo(&nh);
    // RobotInfo robot_info = RobotInfo(&nh, "A", "B", "C", "D");
    
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        robot_info.publish_data();

        ros::spinOnce();

        loop_rate.sleep();
    }
   
    return 0;
} // end function main