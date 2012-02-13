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
#include <pcl_ros/pcl_nodelet.h>



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



class DynamicGeometryMapNode: public pcl_ros::PCLNodelet
{
public:

  // Constructor
	DynamicGeometryMapNode()
  {
//    polygon_sub_ = n_.subscribe("/geometry_map/geometry_map", 1, &DynamicGeometryMapNode::geometryMapCallback, this);
//    get_fov_srv_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetFieldOfView>("get_fov");

  }

  // Destructor
  ~DynamicGeometryMapNode()
  {
    /// void
  }
  void
    onInit()
    {
      PCLNodelet::onInit();
      n_ = getNodeHandle();
      polygon_sub_ = n_.subscribe("/geometry_map/geometry_map", 1, &DynamicGeometryMapNode::geometryMapCallback, this);
      get_fov_srv_client_ = n_.serviceClient<cob_3d_mapping_msgs::GetFieldOfView>("get_fov");
    }


void
geometryMapCallback(const cob_3d_mapping_msgs::PolygonArray::ConstPtr p)
{
	std::cout << "drin";

	cob_3d_mapping_msgs::GetFieldOfView get_fov_srv;
    get_fov_srv.request.target_frame = std::string("/map");


    Eigen::Vector3d n_up,n_up_inter_origin,n_up_inter;
    Eigen::Vector3d n_down,n_down_inter_origin,n_down_inter;
    Eigen::Vector3d n_right,n_right_inter_origin,n_right_inter;
    Eigen::Vector3d n_left,n_left_inter_origin,n_left_inter;
    Eigen::Vector3d n_origin;
    Eigen::Vector3d n_max_range,n_max_range_inter_origin,n_max_range_inter;


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


	MapEntryPtr map_entry_ptr = MapEntryPtr(new MapEntry());
    convertFromROSMsg(*p, *map_entry_ptr);

    //deklarieren von normal und d über map_entry
    std::cout << "n up" << n_up << std::endl;
    std::cout << "p normal " << p->normal << std::endl;

    double d;
    Eigen::Vector3d normal;
    std::vector<bool>  intersec;
    std::vector<Eigen::Vector3f> inter_point;

    intersec.resize(8);
    inter_point.resize(8);

    calcIntersectionLine(n_up ,n_origin,normal,d,n_up_inter_origin,n_up_inter);
    calcIntersectionLine(n_down ,n_origin,normal,d,n_down_inter_origin,n_down_inter);
    calcIntersectionLine(n_right ,n_origin,normal,d,n_right_inter_origin,n_right_inter);
    calcIntersectionLine(n_left ,n_origin,normal,d,n_left_inter_origin,n_left_inter);
    calcIntersectionLine(n_max_range ,n_origin,normal,d,n_max_range_inter_origin,n_max_range_inter); // Fehler n_origin stimmt nicht

    intersec[0]=intersection2Lines(n_up_inter_origin,n_up_inter,n_left_inter_origin,n_left_inter,inter_point[0]);
    intersec[1]=intersection2Lines(n_up_inter_origin,n_up_inter,n_max_range_inter_origin,n_max_range_inter,inter_point[1]);
    intersec[2]=intersection2Lines(n_up_inter_origin,n_up_inter,n_right_inter_origin,n_right_inter,inter_point[2]);
    intersec[3]=intersection2Lines(n_left_inter_origin,n_left_inter,n_max_range_inter_origin,n_max_range_inter,inter_point[3]);
    intersec[4]=intersection2Lines(n_left_inter_origin,n_left_inter,n_down_inter_origin,n_down_inter,inter_point[4]);
    intersec[5]=intersection2Lines(n_max_range_inter_origin,n_max_range_inter,n_down_inter_origin,n_down_inter,inter_point[5]);
    intersec[6]=intersection2Lines(n_max_range_inter_origin,n_max_range_inter,n_right_inter_origin,n_right_inter,inter_point[6]);
    intersec[7]=intersection2Lines(n_down_inter_origin,n_down_inter,n_right_inter_origin,n_right_inter,inter_point[7]);

    int sum;
    std::vector<int> intersections;
	for (int i=0; i<8;i++)
	{
		sum +=intersec[i];
		if(intersec[i]==1){
		intersections.push_back(i);}
	}

	MapEntry fovplan;
	fovplan.polygon_world.resize(1);


	switch (sum)
	{
	case 0: return;break;
	case 3:	fovplan.polygon_world[0].resize(3);
		    fovplan.polygon_world[0][0]=inter_point[intersections[0]];
		    fovplan.polygon_world[0][1]=inter_point[intersections[1]];
		    fovplan.polygon_world[0][2]=inter_point[intersections[2]];

	case 4:	fovplan.polygon_world[0].resize(4);
			fovplan.polygon_world[0][0]=inter_point[intersections[0]];
			fovplan.polygon_world[0][1]=inter_point[intersections[1]];
			fovplan.polygon_world[0][2]=inter_point[intersections[2]];
			fovplan.polygon_world[0][3]=inter_point[intersections[3]];

	default: std::cout << "error wrong number of intersections" << std::endl;

	}

//	gpc_polygon gpc_fov_entry;
//	gpc_polygon gpc_result;
//	gpc_polygon gpc_p_map;
//
//	getGpcStructureUsingMap(fovplan, p_map.transformation_from_world_to_plane, &gpc_fov_entry);
//	getGpcStructureUsingMap(p_map, p_map.transformation_from_world_to_plane, &gpc_p_map);
//
//
//    gpc_polygon_clip(GPC_INT, &gpc_p_merge, &gpc_p_map, &gpc_result);
//    if(gpc_result.num_contours == 0)
//	  {
//		continue;
//	  }
//
//    gpc_polygon_clip(GPC_UNION, &gpc_fov_entry, &gpc_p_map, &gpc_result);
//
//    MapEntry& p_fov; //deklarieren von normale etc
//
//    p_fov.polygon_world.resize(gpc_result.num_contours);
//
//    	for(int j=0; j<gpc_result.num_contours; j++)
//    	{
//    		p_fov.polygon_world[j].resize(gpc_result.contour[j].num_vertices);
//
//    	  for(int k=0; k<gpc_result.contour[j].num_vertices; k++)
//    	  {
//    		//TODO: set z to something else?
//    		Eigen::Vector3f point(gpc_result.contour[j].vertex[k].x, gpc_result.contour[j].vertex[k].y, 0);
//    		p_fov.polygon_world[j][k] = p_fov.transform_from_world_to_plane.inverse()*point;
//    		//TODO: update normal, d, transformation...?
//    	  }
//    	}
//        fov_map_.push_back(p_fov);

 }

void
calcIntersectionLine(Eigen::Vector3d n , Eigen::Vector3d origin ,Eigen::Vector3d normal , double d , Eigen::Vector3d& origin_line , Eigen::Vector3d& direction_line )
{
	direction_line = n.cross(normal);

	if (direction_line(0)*direction_line(0)+direction_line(1)*direction_line(1)+direction_line(2)*direction_line(2)==0)
	{
		origin_line<<0,0,0;
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
intersection2Lines(Eigen::Vector3d orgigin_ln1,Eigen::Vector3d ln1,Eigen::Vector3d orgigin_ln2,Eigen::Vector3d ln2,Eigen::Vector3f& intersection)
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

void
  convertFromROSMsg(const cob_3d_mapping_msgs::PolygonArray& p, MapEntry& map_entry)
  {
    map_entry.id = 0;
    map_entry.d = p.d.data;
    map_entry.normal(0) = p.normal.x;
    map_entry.normal(1) = p.normal.y;
    map_entry.normal(2) = p.normal.z;
    map_entry.merged = 0;
    //map_entry.polygon_world.resize(p.polygons.size());
    for(unsigned int i=0; i<p.polygons.size(); i++)
    {
      if(p.polygons[i].points.size())
      {
        std::vector<Eigen::Vector3f> pts;
        pts.resize(p.polygons[i].points.size());
        for(unsigned int j=0; j<p.polygons[i].points.size(); j++)
        {
          /*pts[j] = Eigen::Vector3f(p.polygons[i].points[j].x,
                                   p.polygons[i].points[j].y,
                                   p.polygons[i].points[j].z);*/
          pts[j](0) = p.polygons[i].points[j].x;
          pts[j](1) = p.polygons[i].points[j].y;
          pts[j](2) = p.polygons[i].points[j].z;
        }
        map_entry.polygon_world.push_back(pts);
      }
    }
  }


ros::NodeHandle n_;


protected:
  ros::Subscriber polygon_sub_;
  ros::ServiceClient get_fov_srv_client_;
  std::vector<MapEntryPtr> fov_map_;


  GeometryMap geometry_map_;


};

/*
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
*/

//PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_geometry_map, DynamicGeometryMapNode, DynamicGeometryMapNode, nodelet::Nodelet)
