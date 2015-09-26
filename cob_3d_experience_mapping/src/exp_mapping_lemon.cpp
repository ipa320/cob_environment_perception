
//visualization is enabled
#define VIS_

#include <cob_3d_experience_mapping/mapping.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <cob_3d_experience_mapping/visualization/graph.hpp>
#include "../include/ros_node.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "exp_mapping");

	ROS_Node<As_Node> sn;
	sn.onInit();

	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();
	
	return 0;
}
