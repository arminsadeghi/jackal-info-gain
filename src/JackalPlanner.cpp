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
    
    ROS_INFO("Creating the initial state of the frenet planner");
    // calc the current location and the heading 
    float current_location_x = _current_state.pose.pose.position.x;
    float current_location_y = _current_state.pose.pose.position.y;
    // // float current_speed_linear_x = _current_state.twist.twist.linear.x;
    // // float current_speed_linear_y = _current_state.twist.twist.linear.y;
    // // float current_speed_angular = _current_state.twist.twist.angular.z;

    tf::Quaternion q(
        _current_state.pose.pose.orientation.x,
        _current_state.pose.pose.orientation.y,
        _current_state.pose.pose.orientation.z,
        _current_state.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    init_con->c_speed = 0;//sqrt(
    // // //     current_speed_linear_x*current_speed_linear_x +
    // // //     current_speed_linear_y*current_speed_linear_y
    // // // );
    init_con->s0 = 0;
    init_con->c_d = 0;//current_speed_angular;
    init_con->c_d_d = 0;
    init_con->c_d_dd = 0;

    int path_length = _global_path.poses.size();
    ROS_INFO("Length of the global path: %d", path_length);
    

    int max_length = std::min(30, path_length);
    int step_size = path_length/max_length;

    init_con->wx = new double[max_length];
    init_con->wy = new double[max_length];
    int counter = 0;
    while ( counter < max_length){
        init_con->wx[counter] = _global_path.poses[counter*step_size].pose.position.x;
        init_con->wy[counter] = _global_path.poses[counter*step_size].pose.position.y;
        counter++;
    }
    init_con->nw = counter;
}

/**
 * @brief callback function for the global path generated from 
 * move_base 
 * 
 * @param _path 
 */
void JackalPlanner::_global_path_cb(const nav_msgs::Path::ConstPtr& msg){

    // get the current global plan
    _global_path = nav_msgs::Path(*msg); 
    ROS_INFO("New global path arrived with length : %ld", _global_path.poses.size());
}

/**
 * @brief publish the generated frenet paths
 * 
 */
void JackalPlanner::publishFrenetPaths(){

    if ((ros::Time::now() - _last_planning_update).toSec() < _replanning_time) {return;}

    if (_global_path.poses.size() < 2) { return;}

    // calc the initial conditions
    FrenetInitialConditions init_con;
    _createFrenetState(&init_con);

    ROS_INFO("Creating the Frenet Paths!");
    // generate the paths 
    
    _frenet_planner->frenet_paths.clear();
    _frenet_planner->calcFrenetPaths(&init_con);
    ROS_INFO("Number of frenet paths: %ld", _frenet_planner->frenet_paths.size());

    
    _last_planning_update = ros::Time::now();

    // publish new paths visualze 
    int n_paths = _frenet_planner->frenet_paths.size();

    jackal_info_gain::FrenetPaths msg;
    msg.header = std_msgs::Header();
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    for (int i=0; i< n_paths; i++){
        int path_length = _frenet_planner->frenet_paths[i]->x.size();
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        for (int j =0; j< path_length; j++){
            geometry_msgs::PoseStamped p;
            p.header.frame_id = "map";
            p.header.stamp = ros::Time::now();
            p.pose.position.x = _frenet_planner->frenet_paths[i]->x[j];
            p.pose.position.y = _frenet_planner->frenet_paths[i]->y[j];
            p.pose.position.z = 0;
            
            path.poses.push_back(p);
        }
        msg.paths.push_back(path);
    }

    // jsk_recognition_msgs::PolygonArray msg;
    // msg.header = std_msgs::Header();
    // msg.header.frame_id = "map";
    // msg.header.stamp = ros::Time::now();
    // for (int i=0; i< n_paths; i++){
    //     int path_length = _frenet_planner->frenet_paths[i]->x.size();
    //     geometry_msgs::PolygonStamped polygon;
    //     polygon.header.frame_id = "map";
    //     polygon.header.stamp = ros::Time::now();
    //     for (int j =0; j< path_length; j++){
    //         geometry_msgs::Point32 p;
    //         p.x = _frenet_planner->frenet_paths[i]->x[j];
    //         p.y = _frenet_planner->frenet_paths[i]->y[j];
    //         p.z = 0;
    //         polygon.polygon.points.push_back(p);
    //     }
    //     msg.polygons.push_back(polygon);
    //     msg.likelihood.push_back(1.0);
    // }
    _pub_paths.publish(msg);
}

/**
 * @brief callback function for odometery
 * 
 * @param msg 
 */
void JackalPlanner::_odom_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    ROS_INFO("New odom message arrived!");
    _current_state.header.frame_id = msg->header.frame_id;
     _current_state.pose.pose.position.x = msg->pose.pose.position.x;
     _current_state.pose.pose.position.y = msg->pose.pose.position.y;

    ROS_INFO("coppied the new odom message!");
}

/**
 * @brief get the frenet hyper parameters
 * 
 * @param fhp 
 */
void JackalPlanner::_getPlannerParams(FrenetHyperparameters& fhp){

    ROS_INFO("Setting the frenet planner params");
    if (_nh->getParam("/jig/max_speed", fhp.max_speed)){
        ROS_INFO("Maximum Speed: %f", fhp.max_speed);
    }
    if (_nh->getParam("/jig/max_accel", fhp.max_accel)){
        ROS_INFO("max_accel: %f", fhp.max_accel);
    }
    if (_nh->getParam("/jig/max_road_width_l", fhp.max_road_width_l)){
        ROS_INFO("max_road_width_l: %f", fhp.max_road_width_l);
    }
    if (_nh->getParam("/jig/max_road_width_r", fhp.max_road_width_r)){
        ROS_INFO("max_road_width_r: %f", fhp.max_road_width_r);
    }
    if (_nh->getParam("/jig/d_road_width", fhp.d_road_w)){
        ROS_INFO("d_road_w: %f", fhp.d_road_w);
    }
    if (_nh->getParam("/jig/dt", fhp.dt)){
        ROS_INFO("dt: %f", fhp.dt);
    }
    if (_nh->getParam("/jig/maxt", fhp.maxt)){
        ROS_INFO("maxt: %f", fhp.maxt);
    }
    if (_nh->getParam("/jig/mint", fhp.mint)){
        ROS_INFO("mint: %f", fhp.mint);
    }
    if (_nh->getParam("/jig/d_t_s", fhp.d_t_s)){
        ROS_INFO("d_t_s: %f", fhp.d_t_s);
    }
    if (_nh->getParam("/jig/n_s_sample", fhp.n_s_sample)){
        ROS_INFO("n_s_sample: %f", fhp.n_s_sample);
    }
    if (_nh->getParam("/jig/obstacle_clearance", fhp.obstacle_clearance)){
        ROS_INFO("obstacle_clearance: %f", fhp.obstacle_clearance);
    }
    if (_nh->getParam("/jig/kv", fhp.kv)){
        ROS_INFO("kv: %f", fhp.kv);
    }
    if (_nh->getParam("/jig/ka", fhp.ka)){
        ROS_INFO("ka: %f", fhp.ka);
    }
    if (_nh->getParam("/jig/kj", fhp.kj)){
        ROS_INFO("kj: %f", fhp.kj);
    }
    if (_nh->getParam("/jig/kt", fhp.kt)){
        ROS_INFO("kt: %f", fhp.kt);
    }
    if (_nh->getParam("/jig/ko", fhp.ko)){
        ROS_INFO("ko: %f", fhp.ko);
    }
    if (_nh->getParam("/jig/klat", fhp.klat)){
        ROS_INFO("klat: %f", fhp.klat);
    }
    if (_nh->getParam("/jig/klon", fhp.klon)){
        ROS_INFO("klon: %f", fhp.klon);
    }
    if (_nh->getParam("/jig/num_threads", fhp.num_threads)){
        ROS_INFO("num_threads: %d", fhp.num_threads);
    }

    if (_nh->getParam("/jig/global_path_topic", _global_path_topic)){
        ROS_INFO("global_path_topic: %s", _global_path_topic.c_str());
    }
    if (_nh->getParam("/jig/odom_sub_topic", _odom_sub_topic)){
        ROS_INFO("odom_sub_topic: %s", _odom_sub_topic.c_str());
    }
    if (_nh->getParam("/jig/frenet_paths_pub_topic", _frenet_paths_pub_topic)){
        ROS_INFO("frenet_paths_pub_topic: %s", _frenet_paths_pub_topic.c_str());
    }
    if (_nh->getParam("/jig/replanning_time", _replanning_time)){
        ROS_INFO("replanning_time: %f", _replanning_time);
    }
    
}