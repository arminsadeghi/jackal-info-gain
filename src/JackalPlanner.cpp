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
#include <ros/console.h>
#include <tf/tf.h>


/**
 * @brief create initial condition for the planner from the ros messages
 * 
 * @param init_con 
 */
void JackalPlanner::_createFrenetState(FrenetInitialConditions* init_con){
    
    // calc the current location and the heading 
    float current_location_x = _current_state.pose.pose.position.x;
    float current_location_y = _current_state.pose.pose.position.y;
    float current_speed_linear_x = _current_state.twist.twist.linear.x;
    float current_speed_linear_y = _current_state.twist.twist.linear.y;
    float current_speed_angular = _current_state.twist.twist.angular.z;

    tf::Quaternion q(
        _current_state.pose.pose.orientation.x,
        _current_state.pose.pose.orientation.y,
        _current_state.pose.pose.orientation.z,
        _current_state.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    init_con->c_speed = sqrt(
        current_speed_linear_x*current_speed_linear_x +
        current_speed_linear_y*current_speed_linear_y
    );
    init_con->s0 = 0;
    init_con->c_d = current_speed_angular;
    init_con->c_d_d = 0;
    init_con->c_d_dd = 0;

    int path_length = sizeof(_global_path.poses)/sizeof(_global_path.poses[0]);
    init_con->wx = new double[path_length];

    for (int i=0; i <path_length; i++){
        init_con->wx[0] = _global_path.poses[i].pose.position.x;
        init_con->wy[0] = _global_path.poses[i].pose.position.y;
    }
    init_con->nw = path_length;
}

/**
 * @brief callback function for the global path generated from 
 * move_base 
 * 
 * @param _path 
 */
void JackalPlanner::_global_path_cb(nav_msgs::PathConstPtr& _path){
    _global_path.header = _path->header;
    _global_path.poses = _path->poses;

    // check if replanning is required
    if ((ros::Time::now() - _last_planning_update).toSec() < _replanning_time) {return;}
    
    // calc the initial conditions
    FrenetInitialConditions init_con;
    _createFrenetState(&init_con);

    // generate the paths 
    _frenet_planner->calcFrenetPaths(&init_con);

    // publish new paths

    _last_planning_update = ros::Time::now();
}

/**
 * @brief publish the generated frenet paths
 * 
 */
void JackalPlanner::_publishFrenetPaths(){

}

/**
 * @brief callback function for odometery
 * 
 * @param msg 
 */
void JackalPlanner::_odom_cb(nav_msgs::OdometryConstPtr& msg){
    _current_state = *msg;
}

/**
 * @brief get the frenet hyper parameters
 * 
 * @param fhp 
 */
void JackalPlanner::_getPlannerParams(FrenetHyperparameters* fhp){

    ROS_INFO("Setting the frenet planner params");
    if (_nh->getParam("/jig/max_speed", fhp->max_speed)){
        ROS_INFO("\tMaximum Speed", std::to_string(fhp->max_speed));
    }
    if (_nh->getParam("/jig/max_accel", fhp->max_accel)){
        ROS_INFO("\tmax_accel", std::to_string(fhp->max_accel));
    }
    if (_nh->getParam("/jig/max_road_width_l", fhp->max_road_width_l)){
        ROS_INFO("\tmax_road_width_l", std::to_string(fhp->max_road_width_l));
    }
    if (_nh->getParam("/jig/max_road_width_r", fhp->max_road_width_r)){
        ROS_INFO("\tmax_road_width_r", std::to_string(fhp->max_road_width_r));
    }
    if (_nh->getParam("/jig/d_road_w", fhp->d_road_w)){
        ROS_INFO("\td_road_w", std::to_string(fhp->d_road_w));
    }
    if (_nh->getParam("/jig/dt", fhp->dt)){
        ROS_INFO("\tdt", std::to_string(fhp->dt));
    }
    if (_nh->getParam("/jig/maxt", fhp->maxt)){
        ROS_INFO("\tmaxt", std::to_string(fhp->maxt));
    }
    if (_nh->getParam("/jig/mint", fhp->mint)){
        ROS_INFO("\tmint", std::to_string(fhp->mint));
    }
    if (_nh->getParam("/jig/d_t_s", fhp->d_t_s)){
        ROS_INFO("\td_t_s", std::to_string(fhp->d_t_s));
    }
    if (_nh->getParam("/jig/n_s_sample", fhp->n_s_sample)){
        ROS_INFO("\tn_s_sample", std::to_string(fhp->n_s_sample));
    }
    if (_nh->getParam("/jig/obstacle_clearance", fhp->obstacle_clearance)){
        ROS_INFO("\tobstacle_clearance", std::to_string(fhp->obstacle_clearance));
    }
    if (_nh->getParam("/jig/kv", fhp->kv)){
        ROS_INFO("\tkv", std::to_string(fhp->kv));
    }
    if (_nh->getParam("/jig/ka", fhp->ka)){
        ROS_INFO("\tka", std::to_string(fhp->ka));
    }
    if (_nh->getParam("/jig/kj", fhp->kj)){
        ROS_INFO("\tkj", std::to_string(fhp->kj));
    }
    if (_nh->getParam("/jig/kt", fhp->kt)){
        ROS_INFO("\tkt", std::to_string(fhp->kt));
    }
    if (_nh->getParam("/jig/ko", fhp->ko)){
        ROS_INFO("\tko", std::to_string(fhp->ko));
    }
    if (_nh->getParam("/jig/klat", fhp->klat)){
        ROS_INFO("\tklat", std::to_string(fhp->klat));
    }
    if (_nh->getParam("/jig/klon", fhp->klon)){
        ROS_INFO("\tklon", std::to_string(fhp->klon));
    }
    if (_nh->getParam("/jig/num_threads", fhp->num_threads)){
        ROS_INFO("\tnum_threads", std::to_string(fhp->num_threads));
    }

    if (_nh->getParam("/jig/global_path_topic", _global_path_topic)){
        ROS_INFO("\tglobal_path_topic", _global_path_topic);
    }
    if (_nh->getParam("/jig/odom_sub_topic", _odom_sub_topic)){
        ROS_INFO("\todom_sub_topic", _odom_sub_topic);
    }
    if (_nh->getParam("/jig/frenet_paths_pub_topic", _frenet_paths_pub_topic)){
        ROS_INFO("\tfrenet_paths_pub_topic", _frenet_paths_pub_topic);
    }
    if (_nh->getParam("/jig/replanning_time", _replanning_time)){
        ROS_INFO("\treplanning_time", std::to_string(_replanning_time));
    }
    
}