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

    FrenetHyperparameters fhp;
    ros::NodeHandle nh;

    ros::spin();
    return 0;
}