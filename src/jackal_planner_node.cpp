/**
 * @file jackal_planner_node.cpp
 * @author Armin Sadeghi (a6sadegh@uwaterloo.ca)
 * @brief 
 * @version 0.1
 * @date 2022-08-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "jackal_info_gain/JackalPlanner.h"
#include "jackal_info_gain/frenet_planner/FrenetStructs.h"



int main(int argc, char** argv){

    ros::init(argc, argv, "jackal_planner");

    ros::NodeHandle nh;
    
    JackalPlanner planner(&nh);
    ros::Rate loop_rate(10);
    while (ros::ok()){
        planner.publishFrenetPaths();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}