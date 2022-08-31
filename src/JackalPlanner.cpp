/**
 * @file JackalPlanner.cpp
 * @author Armin Sadeghi (a6sadegh@uwaterloo.ca)
 * @brief 
 * @version 0.1
 * @date 2022-08-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "jackal_info_gain/JackalPlanner.h"
#include <std_msgs/Header.h>
#include <ros/console.h>
#include <tf/tf.h>


/**
 * @brief create initial condition for the planner from the ros messages
 * 
 * @param init_con 
 */
void JackalPlanner::_createFrenetState(FrenetInitialConditions* init_con){
    
    ROS_INFO("Creating the intial state of the frenet planner");
    // calc the current location and the heading 
    float current_location_x = _current_state.pose.pose.position.x;
    float current_location_y = _current_state.pose.pose.position.y;
    // float current_speed_linear_x = _current_state.twist.twist.linear.x;
    // float current_speed_linear_y = _current_state.twist.twist.linear.y;
    // float current_speed_angular = _current_state.twist.twist.angular.z;

    tf::Quaternion q(
        _current_state.pose.pose.orientation.x,
        _current_state.pose.pose.orientation.y,
        _current_state.pose.pose.orientation.z,
        _current_state.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    init_con->c_speed = 0;//sqrt(
    // //     current_speed_linear_x*current_speed_linear_x +
    // //     current_speed_linear_y*current_speed_linear_y
    // // );
    init_con->s0 = 0;
    init_con->c_d = 0;//current_speed_angular;
    init_con->c_d_d = 0;
    init_con->c_d_dd = 0;

    int path_length = _global_path.poses.size();
    std::cout<<path_length<<","<<std::endl;
    

    int step_size = path_length/30;

    init_con->wx = new double[30];
    init_con->wy = new double[30];
    int counter = 0;
    for (int i=0; i<path_length; i+= step_size){
        init_con->wx[i] = _global_path.poses[i].pose.position.x;
        init_con->wy[i] = _global_path.poses[i].pose.position.y;
        counter ++;
    }
    init_con->nw = path_length;
    ROS_INFO("Creating the intial state of the frenet planner: Done!");
}

/**
 * @brief callback function for the global path generated from 
 * move_base 
 * 
 * @param _path 
 */
void JackalPlanner::_global_path_cb(const nav_msgs::Path::ConstPtr& msg){

    ROS_INFO("New global path arrived!");
    // get the current global plan
    _global_path = nav_msgs::Path(*msg); 


    // check if replanning is required
    if ((ros::Time::now() - _last_planning_update).toSec() < 1) {return;}
    
    // calc the initial conditions
    FrenetInitialConditions init_con;
    _createFrenetState(&init_con);

    ROS_INFO("Creating the Frenet Paths!");
    // generate the paths 
    
    _frenet_planner->frenet_paths.clear();
    _frenet_planner->calcFrenetPaths(&init_con);
    ROS_INFO("Creating the Frenet Paths Done!");
    ROS_INFO("Number of frenet paths: %d", _frenet_planner->frenet_paths.size());

    // // publish new paths

    _last_planning_update = ros::Time::now();
}

/**
 * @brief publish the generated frenet paths
 * 
 */
void JackalPlanner::_publishFrenetPaths(){

    int n_paths = _frenet_planner->frenet_paths.size();

    jackal_info_gain::FrenetPaths msg;
    msg.header = std_msgs::Header();
    msg.header.frame_id = "/map";
    // msg.paths = new nav_msgs::Path[n_paths];
    // for (int i=0; i< n_path; i++){
    //     int path_length = _frenet_planner.frenet_paths[i]->x.size()
    //     for (int j =0; j< path_length; j++){
    //         msg.paths[i].header = std_msgs::Header();
    //         msg.paths[i].header.frame_id = "map";
    //         msg.paths[i].poses.pose.position.x = _frenet_planner.frenet_paths[i]->x[j];
    //         msg.paths[i].poses.pose.position.y = _frenet_planner.frenet_paths[i]->y[j];
    //         msg.paths[i].poses.pose.position.z = 0;
    //     }
    // }
}

/**
 * @brief callback function for odometery
 * 
 * @param msg 
 */
void JackalPlanner::_odom_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    ROS_INFO("New odom message arrived!");
    _current_state = geometry_msgs::PoseWithCovarianceStamped(*msg);
}

/**
 * @brief get the frenet hyper parameters
 * 
 * @param fhp 
 */
void JackalPlanner::_getPlannerParams(FrenetHyperparameters* fhp){

    ROS_INFO("Setting the frenet planner params");
    if (_nh->getParam("/jig/max_speed", fhp->max_speed)){
        // ROS_INFO("\tMaximum Speed", std::to_string(fhp->max_speed));
    }
    if (_nh->getParam("/jig/max_accel", fhp->max_accel)){
        // ROS_INFO("\tmax_accel", std::to_string(fhp->max_accel));
    }
    if (_nh->getParam("/jig/max_road_width_l", fhp->max_road_width_l)){
        // ROS_INFO("\tmax_road_width_l", std::to_string(fhp->max_road_width_l));
    }
    if (_nh->getParam("/jig/max_road_width_r", fhp->max_road_width_r)){
        // ROS_INFO("\tmax_road_width_r", std::to_string(fhp->max_road_width_r));
    }
    if (_nh->getParam("/jig/d_road_width", fhp->d_road_w)){
        // ROS_INFO("\td_road_w", std::to_string(fhp->d_road_w));
    }
    if (_nh->getParam("/jig/dt", fhp->dt)){
        // ROS_INFO("\tdt", std::to_string(fhp->dt));
    }
    if (_nh->getParam("/jig/maxt", fhp->maxt)){
        // ROS_INFO("\tmaxt", std::to_string(fhp->maxt));
    }
    if (_nh->getParam("/jig/mint", fhp->mint)){
        // ROS_INFO("\tmint", std::to_string(fhp->mint));
    }
    if (_nh->getParam("/jig/d_t_s", fhp->d_t_s)){
        // ROS_INFO("\td_t_s", std::to_string(fhp->d_t_s));
    }
    if (_nh->getParam("/jig/n_s_sample", fhp->n_s_sample)){
        // ROS_INFO("\tn_s_sample", std::to_string(fhp->n_s_sample));
    }
    if (_nh->getParam("/jig/obstacle_clearance", fhp->obstacle_clearance)){
        // ROS_INFO("\tobstacle_clearance", std::to_string(fhp->obstacle_clearance));
    }
    if (_nh->getParam("/jig/kv", fhp->kv)){
        // ROS_INFO("\tkv", std::to_string(fhp->kv));
    }
    if (_nh->getParam("/jig/ka", fhp->ka)){
        // ROS_INFO("\tka", std::to_string(fhp->ka));
    }
    if (_nh->getParam("/jig/kj", fhp->kj)){
        // ROS_INFO("\tkj", std::to_string(fhp->kj));
    }
    if (_nh->getParam("/jig/kt", fhp->kt)){
        // ROS_INFO("\tkt", std::to_string(fhp->kt));
    }
    if (_nh->getParam("/jig/ko", fhp->ko)){
        // ROS_INFO("\tko", std::to_string(fhp->ko));
    }
    if (_nh->getParam("/jig/klat", fhp->klat)){
        // ROS_INFO("\tklat", std::to_string(fhp->klat));
    }
    if (_nh->getParam("/jig/klon", fhp->klon)){
        // ROS_INFO("\tklon", std::to_string(fhp->klon));
    }
    if (_nh->getParam("/jig/num_threads", fhp->num_threads)){
        // ROS_INFO("\tnum_threads", std::to_string(fhp->num_threads));
    }

    if (_nh->getParam("/jig/global_path_topic", _global_path_topic)){
        // ROS_INFO("\tglobal_path_topic", _global_path_topic);
    }
    if (_nh->getParam("/jig/odom_sub_topic", _odom_sub_topic)){
        // ROS_INFO("\todom_sub_topic", _odom_sub_topic);
    }
    if (_nh->getParam("/jig/frenet_paths_pub_topic", _frenet_paths_pub_topic)){
        // ROS_INFO("\tfrenet_paths_pub_topic: %s", _frenet_paths_pub_topic);
        std::cout<<"_frenet_paths_pub_topic: "<<_frenet_paths_pub_topic<<endl;
    }
    if (_nh->getParam("/jig/replanning_time", _replanning_time)){
        // ROS_INFO("\treplanning_time", std::to_string(_replanning_time));
    }
    
}