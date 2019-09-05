/*
    Created by Nathan on 9/1/2019
*/


//std imports
#include <iostream>
#include <vector>
#include <string.h>

//ros imports
#include "ros/ros.h"


//


class TrackingNode {
    public:
        TrackingNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

        double* getObjectPosition(int objectId);
        double* getObject_linearPrediction(int objectId);
        double* getObject_polynomialPrediction(int objectId);


        double* evaluatePrediction_linear(double* prediction, double t);
        double* evaluatePrediction_polynomial(double * prediction, double t);

    private:
        ros::Subscriber
        tf::TransformListener tf_listener;

        double* boundingBoxCB(const AprilTag::msg::ConstPtr &bbMsg);
        double* findPosition();
        double* findPrediction_polynomial(int objectId);
        double* findPrediction_linear(int objectId);

        std::vector<std::vector<double*> historicalPositions;
        std::vector<std::string> objectMapping; //indexing for objects by unique id (whatever apriltags passes)

        //parameters 
        //subscriber topics
        std::string boundingBox_topic;
        std::string depth_topic;

        //publisher topics
        std::string prediction_topic;
        std::string position_topic;

        //TODO find rotation from depth
        enum object_depth_types { AVERAGE, MEDIAN }
        int depth_algorithm;
}