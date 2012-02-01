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
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


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

 //   boost::shared_ptr<std::vector<MapEntryPtr> > map= geometry_map_.getMap();
//
//    for (int i=0 ; i<map->size();i++)
//    {
//    	MapEntry& pm = *(map->at(i));
//    	Eigen::Vector3d normal=pm.normal;
//    	double d=pm.d;

//   	    Eigen::Vector3d intersec_up =normal.cross(n_up);
//    	Eigen::Vector3fd intersec_down =normal.cross(n_down);
//    	Eigen::Vector3f intersec_left =normal.cross(n_left);
//    	Eigen::Vector3f intersec_right =normal.cross(n_right);
//    	Eigen::Vector3f intersec_max_range =normal.cross(n_max_range);



 }

void
calcIntersectionLine(Eigen::Vector3d n , Eigen::Vector3d origin ,Eigen::Vector3d normal , double d , Eigen::Vector3d& origin_line , Eigen::Vector3d& direction_line )
{
	direction_line = n.cross(normal);
	if (direction_line(0)*direction_line(0)+direction_line(1)*direction_line(1)+direction_line(2)*direction_line(2)==0)
	{
		return;
	}
	double d_n=-n(0)*origin(0)-n(1)*origin(1)-n(2)*origin(2);
	double max = n(0)+normal(0);
	int direction=0;
	if(n(1)+normal(1)>max)
	{
		max=n(1)+normal(1);
		direction=1;
	}
	if(n(2)+normal(2)>max)
	{
		max=n(2)+normal(2);
		direction=2;
	}
	origin_line << 0,0,0;
	origin_line(direction)=(d-d_n)/(n(direction)-normal(direction));


}

bool
intersection2Lines(Eigen::Vector3d orgigin_ln1,Eigen::Vector3d ln1,Eigen::Vector3d orgigin_ln2,Eigen::Vector3d ln2,Eigen::Vector3d& intersection)
{
	Eigen::Matrix2d matrix;
	Eigen::Vector2d vec;

		if(ln1(0)==0 && ln2(0)==0)
		{
			matrix(0,0)=ln1(1);
			matrix(1,0)=ln1(2);
			matrix(0,1)=-ln2(1);
			matrix(1,1)=-ln2(2);


			vec << orgigin_ln2(1)-orgigin_ln1(1),orgigin_ln2(2)-orgigin_ln1(2);
			Eigen::Vector3d x = matrix.colPivHouseholderQr().solve(vec);
			//std::cout << "x" << x(0) << "," << x(1)  << std::endl;
			if(abs(ln1(0)*x(0)+orgigin_ln1(0)-ln2(0)*x(1)-orgigin_ln2(0))<0.001)
			{
				intersection << ln1(0)*x(0)+orgigin_ln1(0),
						ln1(1)*x(0)+orgigin_ln1(1),
						ln1(2)*x(0)+orgigin_ln1(2);
				return true;
			}
			return false;




		}
		else
		{
			std::cout << "ln1:" << ln1(2) << std::endl;

			matrix(0,0)=ln1(0);
			matrix(1,0)=ln1(1);
			matrix(0,1)=-ln2(0);
			matrix(1,1)=-ln2(1);


			vec << orgigin_ln2(0)-orgigin_ln1(0),orgigin_ln2(1)-orgigin_ln1(1);
			Eigen::Vector3d x = matrix.colPivHouseholderQr().solve(vec);
			//std::cout << "x" << x(0) << "," << x(1)  << std::endl;
			//std::cout << "term   " << ln1(2) * x(0)+orgigin_ln1(2)-ln2(2)*x(1)-orgigin_ln2(2) << std::endl;
			if(abs(ln1(2) * x(0)+orgigin_ln1(2)-ln2(2)*x(1)-orgigin_ln2(2))<=0.001)
			{
				intersection << ln1(0)*x(0)+orgigin_ln1(0),
						ln1(1)*x(0)+orgigin_ln1(1),
						ln1(2)*x(0)+orgigin_ln1(2);
				return true;
			}
			return false;


		}



}
ros::NodeHandle n_;


protected:
  ros::Subscriber polygon_sub_;
  ros::ServiceClient get_fov_srv_client_;

  GeometryMap geometry_map_;


};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "dynamic_geometry_map_node");

  DynamicGeometryMapNode dgmn;
  Eigen::Vector3d n;
  n << 0,0,1;
  Eigen::Vector3d origin;
  origin << 0,0,0;
  Eigen::Vector3d normal;
  normal << 0,1,1;
  Eigen::Vector3d origin2;
  origin2 << 0,1,1;
  Eigen::Vector3d normal2;
  normal2 << 0,2,4;
  bool x;
  Eigen::Vector3d inter;


  x=dgmn.intersection2Lines(origin,normal,origin2,normal2,inter);
  std::cout << inter << "intersec " << x;
// dgmn.calcIntersectionLine(n,origin,normal,d,origin_line,direction_line);

//  ros::Rate loop_rate(10);
//  while (ros::ok())
//  {
//    ros::spinOnce ();
//    loop_rate.sleep();
  }


