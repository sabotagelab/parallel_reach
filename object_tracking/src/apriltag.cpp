#include "tracking_node.hpp"


int main(int argc, char** argv) {
    ros::init(argc, argv, "Apriltag_Object_Tracking");
    ros::NodeHandle nh;
    TrackingNode tracking_node(nh);

    ROS_INFO("STARTED APRILTAGS NODE")
    //TODO properly start node
    ros::spin();
    return 0;
}