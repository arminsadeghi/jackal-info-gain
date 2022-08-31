/**
 * @file jackal_planner.h
 * @author Armin Sadeghi (a6sadegh@uwaterloo.ca)
 * @brief 
 * @version 0.1
 * @date 2022-08-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef JACKAL_INFO_JACKAL_PLANNER
#define JACKAL_INFO_JACKAL_PLANNER

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <jackal_info_gain/FrenetPaths.h>
#include <iostream>
#include "jackal_info_gain/frenet_planner/FrenetOptimalTrajectory.h"

class JackalPlanner {
    ros::NodeHandle* _nh;
    ros::Subscriber _sub_global_path;
    ros::Subscriber _sub_odom;
    ros::Publisher _pub_paths;
    nav_msgs::Path _global_path;
    std::string _global_path_topic;
    std::string _odom_sub_topic;
    std::string _frenet_paths_pub_topic;
    nav_msgs::Odometry _current_state;
    double _replanning_time;
    FrenetOptimalTrajectory* _frenet_planner;
    ros::Time _last_planning_update;

    void _global_path_cb(nav_msgs::PathConstPtr&);
    void _odom_cb(nav_msgs::OdometryConstPtr&);
    void _getTrajectories();
    void _getPlannerParams(FrenetHyperparameters*);
    void _createFrenetState(FrenetInitialConditions*);
    void _publishFrenetPaths();
    
    public:
    JackalPlanner(ros::NodeHandle* nh){
        
        _nh = nh;

        FrenetHyperparameters fhp;
        _getPlannerParams(&fhp);

        _frenet_planner = new FrenetOptimalTrajectory(&fhp);

        // subscribe to the global path e.g. move base path
        _sub_global_path = nh->subscribe(
            _global_path_topic,
            1000,
            &JackalPlanner::_global_path_cb,
            this
        );

        // subscribe to the current state of the robot
        _sub_odom = nh->subscribe(
            _odom_sub_topic,
            1000,
            &JackalPlanner::_odom_cb,
            this
        );

        _pub_paths = nh->advertise<jackal_info_gain::FrenetPaths>(
            _frenet_paths_pub_topic,
            1000
        );

        _last_planning_update = ros::Time::now();
    }

};
#endif //JACKAL_INFO_JACKAL_PLANNER