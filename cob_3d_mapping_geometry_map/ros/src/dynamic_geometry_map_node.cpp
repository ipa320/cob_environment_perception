//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transform.h>
#include <cob_3d_mapping_common/reconfigureable_node.h>
#include <cob_3d_mapping_msgs/GetFieldOfView.h>


#include <cob_3d_mapping_geometry_map/geometry_map_nodeConfig.h>

#include "pcl/surface/convex_hull.h"
#include "pcl/filters/project_inliers.h"


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_3d_mapping_msgs/PolygonArray.h>
#include <cob_3d_mapping_msgs/PolygonArrayArray.h>
#include <cob_3d_mapping_msgs/GetGeometricMap.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/GetFieldOfView.h>

#include <geometry_msgs/PolygonStamped.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>

// internal includes
#include "cob_3d_mapping_geometry_map/geometry_map.h"
#include "cob_3d_mapping_geometry_map/map_entry.h"



class DynamicGeometryMapNode
{
public:

  // Constructor
	DynamicGeometryMapNode()
  {
    polygon_sub_ = n_.subscribe("/geometry_map/geometry_map_2", 1, &DynamicGeometryMapNode::geometryMapCallback, this);
    get_fov_srv_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetFieldOfView>("get_fov");

  }

  // Destructor
  ~DynamicGeometryMapNode()
  {
    /// void
  }

void
geometryMapCallback(const cob_3d_mapping_msgs::ShapeArray& map)
{
	std::cout << "drin " << std::endl;
	cob_3d_mapping_msgs::GetFieldOfView get_fov_srv;

    get_fov_srv.request.target_frame = std::string("/map");

    Eigen::Vector3d n_up;
    Eigen::Vector3d n_down;
    Eigen::Vector3d n_right;
    Eigen::Vector3d n_left;
    Eigen::Vector3d n_origin;
    Eigen::Vector3d n_max_range;

    if(get_fov_srv_client_.call(get_fov_srv))
    {
        n_up(0) = get_fov_srv.response.fov.points[0].x;
        n_up(1) = get_fov_srv.response.fov.points[0].y;
        n_up(2) = get_fov_srv.response.fov.points[0].z;
        n_down(0) = get_fov_srv.response.fov.points[1].x;
        n_down(1) = get_fov_srv.response.fov.points[1].y;
        n_down(2) = get_fov_srv.response.fov.points[1].z;
        n_right(0) = get_fov_srv.response.fov.points[2].x;
        n_right(1) = get_fov_srv.response.fov.points[2].y;
        n_right(2) = get_fov_srv.response.fov.points[2].z;
        n_left(0) = get_fov_srv.response.fov.points[3].x;
        n_left(1) = get_fov_srv.response.fov.points[3].y;
        n_left(2) = get_fov_srv.response.fov.points[3].z;
        n_origin(0) = get_fov_srv.response.fov.points[4].x;
        n_origin(1) = get_fov_srv.response.fov.points[4].y;
        n_origin(2) = get_fov_srv.response.fov.points[4].z;
        n_max_range(0) = get_fov_srv.response.fov.points[5].x;
        n_max_range(1) = get_fov_srv.response.fov.points[5].y;
        n_max_range(2) = get_fov_srv.response.fov.points[5].z;

    }
 }

ros::NodeHandle n_;


protected:
  ros::Subscriber polygon_sub_;
  ros::ServiceClient get_fov_srv_client_;

};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "dynamic_geometry_map_node");

  DynamicGeometryMapNode dgmn;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep();
  }
}

