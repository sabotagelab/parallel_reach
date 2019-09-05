#include "tracking_node.hpp"



TrackingNode::TrackingNode(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    nh.getParam("boundingbox_topic", this->boundingBox_topic);
    nh.getParam("depth_topic", this->depth_topic);
    nh.getParam("prediction_topic", this->prediction_topic);
    nh.getParam("position_topic", this->position_topic)

    this->boundingBox_sub = nh.subscribe(this->boundingBox_topic, 1, &TrackingNode::boundingBoxCB, this);
    this->depth_sub = nh.subscribe(this->depth_topic, 1, &TrackingNode::StoreDepthMsg, this);

    this->prediction_pub = nh.advertise<osuf1_common::StampedFloat2d>(this->prediction_topic, 1);
    this->position_pub = nh.advertise<osuf1_common::StampedFloat2d>(this->position_topic, 1);
}


double * TrackingNode::findPosition(int obj) {

}