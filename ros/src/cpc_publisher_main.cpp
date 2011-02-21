/*
 * cpc_publisher_main.cpp
 *
 *  Created on: 25.08.2010
 *      Author: goa
 */

// ROS includes
#include <ros/ros.h>
#include <cob_vision_ipa_utils/cpc_publisher.h>

// ROS service includes
//--

// external includes
//--


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, specify name of node
    ros::init(argc, argv, "cpc_publisher");

    PcPublisher pcPublisher;
    pcPublisher.initNode();

    ros::spin();

    return 0;
}
