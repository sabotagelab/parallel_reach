//
// Created by Niraj on 6/23/19.
//

#include <iostream>
#include <fstream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "MPC_Kinematic.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

using namespace std;
using namespace Eigen;


class MPCKinematicNode {
public:
    MPCKinematicNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
    int get_thread_numbers();

private:
    //MPC class and parameters
    MPC_Kinematic mpc_;
    MPC_parameters mpc_params_;

    //Odometry and motion variables
    double current_yaw_,current_pos_x_,current_pos_y_;
    geometry_msgs::Point goal_pos_;
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped localized_pose_;
    nav_msgs::Path odom_path_, mpc_traj_ , total_path_, local_path_;
    ackermann_msgs::AckermannDriveStamped ackermann_msg_;
    geometry_msgs::Twist twist_msg_;

    //Topic names and tf frames
    string _globalPath_topic, _goal_topic, cmd_vel_topic, localized_pose_topic;
    string _map_frame, _odom_frame, _car_frame;

    //Variables and parameters
    string waypoint_filename_;
    double _Lf, _dt, _steering, _throttle, _speed, _max_speed;
    double _pathLength, _waypoint_fov, _goalRadius, _waypointsDist;
    int _controller_freq, _downSampling, _thread_numbers;
    bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;
    ros::Timer controller_timer_;

    //Publishers and subscribers
    ros::Subscriber odom_sub_, path_sub_, goal_sub_, localized_pose_sub_;
    ros::Publisher odompath_pub_, twist_pub_, ackermann_pub_, mpctraj_pub_;
    tf::TransformListener tf_listener_;

    //Callback functions
    void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
//    void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclMsg);
    void amclCB(const geometry_msgs::PoseStamped::ConstPtr &amclMsg);
    void controlLoopCB(const ros::TimerEvent &);

    //Polynomial fitting functions
    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
    vector<int> get_nearby_waypoints();
    void read_waypoints_from_csv(nav_msgs::Path& sample_path, string& filename, char delim=',');



}; // end of class


MPCKinematicNode::MPCKinematicNode(ros::NodeHandle nh, ros::NodeHandle private_nh) {

    private_nh.param<std::string>("waypoints_filepath",waypoint_filename_,"/home/houssam/racing/f110_ws/src/ppcm/mpc_racecar/waypoints/kelley_third_floor.csv");
    //Parameter for MPC solver
    private_nh.param("mpc_steps", mpc_params_.mpc_steps_, 20);
    private_nh.param("mpc_ref_cte", mpc_params_.ref_cte_, 0.0);
    private_nh.param("mpc_ref_epsi", mpc_params_.ref_epsi_, 0.0);
    private_nh.param("mpc_ref_vel", mpc_params_.ref_vel_, 1.25);
    private_nh.param("mpc_w_cte", mpc_params_.w_cte_, 2500.0);
    private_nh.param("mpc_w_epsi", mpc_params_.w_epsi_, 2000.0);
    private_nh.param("mpc_w_vel", mpc_params_.w_vel_, 300.0);
    private_nh.param("mpc_w_delta", mpc_params_.w_delta_, 100.0);
    private_nh.param("mpc_w_accel", mpc_params_.w_accel_, 50.0);
    private_nh.param("mpc_w_turn", mpc_params_.w_turn_, 50.0);
    private_nh.param("mpc_w_delta_d", mpc_params_.w_delta_d_, 200.0);
    private_nh.param("mpc_w_accel_d", mpc_params_.w_accel_d_, 40.0);
    private_nh.param("mpc_max_steering", mpc_params_.max_steering_, 0.523); // Maximal steering radian (~30 deg)
    private_nh.param("mpc_max_throttle", mpc_params_.max_throttle_, 1.0); // Maximal throttle accel
    private_nh.param("mpc_bound_value", mpc_params_.bound_value_, 1.0e3); // Bound value for other variables

    //Parameters for control loop
    private_nh.param("thread_numbers", _thread_numbers, 3); // number of threads for this ROS node
    private_nh.param("pub_twist_cmd", _pub_twist_flag, true);
    private_nh.param("debug_info", _debug_info, false);
    private_nh.param("delay_mode", _delay_mode, true);
    private_nh.param("max_speed", _max_speed, 1.5); // unit: m/s
    private_nh.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
    private_nh.param("path_length", _pathLength, 3.0); // unit: m
    private_nh.param("waypoints_fov", _waypoint_fov, 2.09); // unit: radians
    private_nh.param("goal_radius", _goalRadius, 0.2); // unit: m
    private_nh.param("controller_freq", _controller_freq, 10);
    private_nh.param("vehicle_Lf", _Lf, 0.325); // distance between the front of the vehicle and its center of gravity

    _dt = double(1.0 / _controller_freq); // time step duration dt in s
    mpc_params_.Lf_ = _Lf;
    mpc_params_.dt_ = _dt;

    //Setting initial parameters for MPC
    mpc_.SetInitialParameters(mpc_params_);
    mpc_.CalculateParameters();

    //Parameter for topics & Frame name
    private_nh.param<std::string>("global_path_topic", _globalPath_topic,
                                  "/move_base/TrajectoryPlannerROS/global_plan");
    private_nh.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal");
    private_nh.param<std::string>("map_frame", _map_frame, "map");
    private_nh.param<std::string>("odom_frame", _odom_frame, "odom");
    private_nh.param<std::string>("car_frame", _car_frame, "base_link");
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/vesc/high_level/ackermann_cmd_mux/input/nav_0");
    private_nh.param<std::string>("localized_pose_topic", localized_pose_topic, "/pf/viz/inferred_pose");

    //Display the parameters
    cout << "\n===== Parameters =====" << endl;
    cout << "pub_twist_cmd: " << _pub_twist_flag << endl;
    cout << "debug_info: " << _debug_info << endl;
    cout << "delay_mode: " << _delay_mode << endl;
    cout << "vehicle_Lf: " << _Lf << endl;
    cout << "frequency: " << _dt << endl;
    cout << "mpc_steps: " << mpc_params_.mpc_steps_ << endl;
    cout << "mpc_ref_vel: " << mpc_params_.ref_vel_ << endl;
    cout << "mpc_w_cte: " << mpc_params_.w_cte_<< endl;
    cout << "mpc_w_epsi: " << mpc_params_.w_epsi_ << endl;
    cout << "mpc_max_steering: " << mpc_params_.max_steering_ << endl;
    cout << "global_path_topic: " << _globalPath_topic << endl;
    cout << "cmd_vel_topic: " << cmd_vel_topic << endl;
    cout << "localized_pose_topic: " << localized_pose_topic << endl;
    cout << "waypoints filename="<<waypoint_filename_<<endl;

    //Publishers and Subscribers
    odom_sub_ = nh.subscribe("/vesc/odom", 1, &MPCKinematicNode::odomCB, this);
    path_sub_ = nh.subscribe(_globalPath_topic, 1, &MPCKinematicNode::pathCB, this);
    goal_sub_ = nh.subscribe(_goal_topic, 1, &MPCKinematicNode::goalCB, this);
    localized_pose_sub_ = nh.subscribe(localized_pose_topic, 5, &MPCKinematicNode::amclCB, this);

    odompath_pub_ = nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC
    mpctraj_pub_ = nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
    ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>(cmd_vel_topic, 1);
    if (_pub_twist_flag)
        twist_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)

    //Timer
    controller_timer_ = nh.createTimer(ros::Duration((1.0) / _controller_freq), &MPCKinematicNode::controlLoopCB,
                                       this); // 10Hz

    //Init variables
    _goal_received = false;
    _goal_reached = false;
    _path_computed = false;
    _throttle = 0.0;
    _steering = 0.0;
    _speed = 0.0;

    ackermann_msg_ = ackermann_msgs::AckermannDriveStamped();
    twist_msg_ = geometry_msgs::Twist();
    mpc_traj_ = nav_msgs::Path();

    read_waypoints_from_csv(total_path_,waypoint_filename_);


}

// Public: return _thread_numbers
int MPCKinematicNode::get_thread_numbers() {
    return _thread_numbers;
}

// Evaluate a polynomial.
double MPCKinematicNode::polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCKinematicNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++)
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// CallBack: Update odometry
void MPCKinematicNode::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    odom_ = *odomMsg;
}

//returns indices of nearby points
vector<int> MPCKinematicNode::get_nearby_waypoints(){
    vector<int> indices;
    local_path_.header.frame_id = _map_frame; // points in car coordinate
    local_path_.header.stamp = ros::Time::now();
    local_path_.poses.clear();
    for (int i = 0; i < total_path_.poses.size(); i++) {
        double dx = total_path_.poses[i].pose.position.x- current_pos_x_;
        double dy = total_path_.poses[i].pose.position.y- current_pos_y_;
        double dist = sqrt(dx * dx + dy * dy);
        double delta_yaw = atan2(dy,dx) - current_yaw_;
        if(((dist<_pathLength) && (fabs(delta_yaw)<_waypoint_fov)) || (dist<1.25)){
            indices.push_back(i);
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = local_path_.header;
            tempPose.pose.position.x = total_path_.poses[i].pose.position.x ;
            tempPose.pose.position.y = total_path_.poses[i].pose.position.y;
            tempPose.pose.orientation.w = 1.0;
            local_path_.poses.push_back(tempPose);
        }
    }
    return indices;
}


//read waypoints from given csv file and return the data in the form of nav_msgs::Path
void MPCKinematicNode::read_waypoints_from_csv(nav_msgs::Path& sample_path, string& filename, char delim){
    fstream fin;
    fin.open(filename, ios::in);
    vector<string> row;
    string line, word, temp;
    sample_path.header.frame_id = _map_frame; // points in car coordinate
    sample_path.header.stamp = ros::Time::now();
    while (getline(fin, line)) {
        row.clear();
        stringstream s(line);
        // read every column data of a row and store it in a string variable, 'word'
        while (getline(s, word, ',')) {
            row.push_back(word);
        }
        geometry_msgs::PoseStamped tempPose;
        tempPose.header = sample_path.header;
        tempPose.pose.position.x = stod(row[0]);
        tempPose.pose.position.y = stod(row[1]);
        tempPose.pose.orientation.w = 1.0;
        sample_path.poses.push_back(tempPose);
        }
    _path_computed = true;
    }

// CallBack: Update path waypoints (conversion to odom frame)
void MPCKinematicNode::pathCB(const nav_msgs::Path::ConstPtr &pathMsg) {
    if (_goal_received && !_goal_reached) {
        nav_msgs::Path odom_path = nav_msgs::Path();
        try {
            //find waypoints distance
            if (_waypointsDist <= 0.0) {
                double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                _waypointsDist = sqrt(dx * dx + dy * dy);
                _downSampling = int(_pathLength / 10.0 / _waypointsDist);
            }

            double total_length = 0.0;
            int sampling = _downSampling;
            // Cut and downsampling the path
            for (int i = 0; i < pathMsg->poses.size(); i++) {
                if (total_length > _pathLength)
                    break;

                if (sampling == _downSampling) {
                    geometry_msgs::PoseStamped tempPose;
//                    tf_listener_.transformPose(_odom_frame, ros::Time(0), pathMsg->poses[i], _map_frame, tempPose);

                    odom_path.poses.push_back(pathMsg->poses[i]);
//                    odom_path.poses.push_back(tempPose);
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist;
                sampling = sampling + 1;
            }

            if (odom_path.poses.size() >= 6) {
                odom_path_ = odom_path; // Path waypoints in odom frame
                _path_computed = true;
                // publish odom path
                odom_path.header.frame_id = _odom_frame;
//                odom_path.header.frame_id = _car_frame;
                odom_path.header.stamp = ros::Time::now();
                odompath_pub_.publish(odom_path);
            }
            //DEBUG
            cout << endl << "N: " << odom_path.poses.size() << endl <<  "Car path[0]: " << odom_path.poses[0] << ", path[N]: " << odom_path_.poses[odom_path_.poses.size()-1] << endl;


        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}

// CallBack: Update goal status
void MPCKinematicNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg) {
    goal_pos_ = goalMsg->pose.position;
    _goal_received = true;
    _goal_reached = false;
    ROS_INFO("Goal Received !");
}


// Callback: Check if the car is inside the goal area or not
void MPCKinematicNode::amclCB(const geometry_msgs::PoseStamped::ConstPtr &amclMsg) {
    localized_pose_ = *amclMsg;
    current_pos_x_=localized_pose_.pose.position.x;
    current_pos_y_=localized_pose_.pose.position.y;
    tf::Pose pose;
    tf::poseMsgToTF(localized_pose_.pose, pose);
    current_yaw_= tf::getYaw(pose.getRotation());
    if (_goal_received) {
        double car2goal_x = goal_pos_.x - current_pos_x_;
        double car2goal_y = goal_pos_.y - current_pos_y_;
        double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
        if (dist2goal < _goalRadius) {
            _goal_reached = true;
            _goal_received = false;
            ROS_INFO("Goal Reached !");
        }
    }
}



// Timer: Control Loop (closed loop nonlinear MPC)
void MPCKinematicNode::controlLoopCB(const ros::TimerEvent &) {
    if (_goal_received && !_goal_reached && _path_computed) //received goal & goal not reached
    {
        ros::Time control_loop_start_time=ros::Time::now();
        nav_msgs::Odometry odom = odom_;

//        // Update system states: X=[x, y, psi, v]
        const double px = current_pos_x_; //pose: odom frame
        const double py = current_pos_y_;
        const double psi = current_yaw_;
        const double v = odom.twist.twist.linear.x; //twist: body fixed frame

        // Update system inputs: U=[steering, throttle]
        const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        const double dt = _dt;
        const double Lf = _Lf;

        // Waypoints related parameters
        vector<int> test = get_nearby_waypoints();
        nav_msgs::Path odom_path = local_path_;
        odompath_pub_.publish(local_path_);



        const int N = odom_path.poses.size(); // Number of waypoints
        const double cospsi = cos(psi);
        const double sinpsi = sin(psi);

        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for (int i = 0; i < N; i++) {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * cospsi + dy * sinpsi;
            y_veh[i] = dy * cospsi - dx * sinpsi;
        }

        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3);

        const double cte = polyeval(coeffs, 0.0);
        const double epsi = atan(coeffs[1]);
        VectorXd state(6);
        if (_delay_mode) {
            // Kinematic model is used to predict vehicle state at the actual
            // moment of control (current time + delay dt)
            const double px_act = v * dt;
            const double py_act = 0;
            const double psi_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt;
            const double cte_act = cte + v * sin(epsi) * dt;
            const double epsi_act = -epsi + psi_act;
            state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
        } else {
            state << 0, 0, 0, v, cte, epsi;
        }

        // Solve MPC Problem
        vector<double> mpc_results = mpc_.Solve(state, coeffs);

        // MPC result (all described in car frame)
        _steering = mpc_results[0]; // radian
        _throttle = mpc_results[1]; // acceleration
        _speed = v + _throttle * dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if (_speed <= (-_max_speed/2.0))
            _speed = -_max_speed/2.0;
//        if (_speed <0.35 && _speed>0.02){
//            _speed = 0.35;
//        }
//        if (_speed>-0.35 && _speed<-0.02 && _throttle<0){
//            _speed = -0.35;
//        }
//        else if(_speed>-0.35 && _speed<-0.02 && _throttle>0){
//            _speed = 0.35;
//        }

        // Display the MPC predicted trajectory
        mpc_traj_ = nav_msgs::Path();
        mpc_traj_.header.frame_id = _car_frame; // points in car coordinate
        mpc_traj_.header.stamp = ros::Time::now();
        for (int i = 0; i < mpc_.mpc_x.size(); i++) {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = mpc_traj_.header;
            tempPose.pose.position.x = mpc_.mpc_x[i];
            tempPose.pose.position.y = mpc_.mpc_y[i];
            tempPose.pose.orientation.w = 1.0;
            mpc_traj_.poses.push_back(tempPose);
        }
        // publish the mpc trajectory
        mpctraj_pub_.publish(mpc_traj_);

        if (_debug_info) {
            cout << "\n\nDEBUG" << endl;
            cout << "psi: " << psi << endl;
            cout << "V: " << v << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_steering: \n" << _steering << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
            cout << "mpc_speed: \n" << mpc_results[2] << endl;
            ros::Time control_loop_end_time=ros::Time::now();
            ros::Duration diff=control_loop_end_time-control_loop_start_time;
            cout << "Control loop time=: \n" << diff << endl;
        }

    } else {
        _steering = 0.0;
        _throttle = 0.0;
        _speed = 0.0;
        if (_goal_reached && _goal_received)
            cout << "Goal Reached !" << endl;
    }

    // publish cmd
    ackermann_msg_.header.frame_id = _car_frame;
    ackermann_msg_.header.stamp = ros::Time::now();
    ackermann_msg_.drive.steering_angle = _steering;
    ackermann_msg_.drive.speed = _speed;
    ackermann_msg_.drive.acceleration = _throttle;
    ackermann_pub_.publish(ackermann_msg_);

    if (_pub_twist_flag) {
        twist_msg_.linear.x = _speed;
        twist_msg_.angular.z = _steering;
        twist_pub_.publish(twist_msg_);
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "MPC_Kinematic_Node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    MPCKinematicNode mpc_node(nh, private_nh);

    ROS_INFO("Waiting for global path msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
