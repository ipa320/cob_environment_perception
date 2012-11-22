/*
 * cylinder_client.cpp
 *
 *  Created on: Jun 28, 2012
 *      Author: goa-tz
 */

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
#ifdef PCL_VERSION_COMPARE
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>
#include <cob_3d_mapping_geometry_map/geometry_map_nodeConfig.h>

#include "pcl/surface/convex_hull.h"
#include "pcl/filters/project_inliers.h"


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_common/cylinder.h"


// internal includes
#include "cob_3d_mapping_geometry_map/geometry_map.h"
#include <cob_3d_mapping_msgs/Shape.h>

using namespace cob_3d_mapping;
//####################
//#### nodelet class ####
class cylinder_client
{
public:

	// Constructor
	cylinder_client()
	{


		map_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray>("SA",1);




	}

	// Destructor
	~cylinder_client()
	{
		/// void
	}

	void
	transformCylinder(Cylinder::Ptr & c_ptr,Eigen::Affine3f& trafo)
	{


		Cylinder & c=*c_ptr;

		for (int i = 0; i < (int) c.contours.size(); ++i) {
			for (int j = 0; j < (int) c.contours[i].size(); ++j) {


				c.contours[i][j]=trafo*c.contours[i][j];


			}
		}

		c.origin_=trafo*c.origin_;

		for (int i = 0; i < 3; ++i) {

			c.sym_axis[i]=trafo.rotation()*c.sym_axis[i];
			//	std::cout<<"axis -"<<i<<" \n"<<c.axes_[i]<<std::endl;
		}
		c.normal=trafo.rotation()*c.normal;

		float roll,pitch,yaw,x,y,z;
		pcl::getTranslationAndEulerAngles(trafo,x,y,z,roll,pitch,yaw);
		//	std::cout<<" x= "<<x<<" y= "<<z<<" z= "<<z<<" roll= "<<roll<<" pitch= "<<pitch<<" yaw= "<<yaw<<std::endl;

		c.assignMembers(c.sym_axis[1], c.sym_axis[2], c.origin_);	//	configure unrolled polygon
	}


	void
	makePolygon(Polygon::Ptr& p1)
	{
		Eigen::Vector3f v;
			std::vector<Eigen::Vector3f> vv;
			p1->id = 1;
			p1->normal << 0.000000,-1.000000,-0.000000;
			p1->d = 0;
			v << 0.500000,0.010000,0.500000;
			vv.push_back(v);
			v << 0.500000,0.010000,-0.500000;
			vv.push_back(v);
			v << -0.500000,0.010000,-0.500000;
			vv.push_back(v);
			v << -0.500000,0.010000,0.500000;
			vv.push_back(v);
			p1->contours.push_back(vv);
			p1->holes.push_back(0);

	}


	void
	makeCylinder(Cylinder::Ptr& c1)
	{


		//####################################################
		//Cylinder #1


		c1->id = 0;

		Eigen::Vector3f x_axis1,y_axis1,z_axis1;
		std::vector<Eigen::Vector3f> axes1;
		Eigen::Vector3f origin1;
		std::vector<Eigen::Vector3f> contour1;
		std::vector<std::vector<Eigen::Vector3f> > contours1;

		Eigen::Vector3f v1;


		x_axis1 << 1,0,0;
		axes1.push_back(x_axis1);

		y_axis1 << 0,0,1;
		axes1.push_back(y_axis1);

		z_axis1 << 0,1,0;
		axes1.push_back(z_axis1);

		c1->axes_=axes1;
		c1->r_=1;



		v1 << -1, 0, 1;
		contour1.push_back(v1);
		v1 << 0, 1, 1;
		contour1.push_back(v1);
		v1 << 1, 0, 1;
		contour1.push_back(v1);
		v1 << 1 ,0 ,-1;
		contour1.push_back(v1);
		v1 << 0, 1, -1;
		contour1.push_back(v1);
		v1 << -1, 0 ,-1;
		contour1.push_back(v1);

		c1->merged=1;
		origin1 << 0,0,0;
		c1->origin_=origin1;

		c1->holes.push_back(0);
		c1->debug_=false;

		contours1.push_back(contour1);
		c1->ContoursFromList(contours1);







//		//####################################################
//		//Cylinder  #2
//		CylinderPtr  c2  =CylinderPtr(new Cylinder());
//		c2->id = 0;
//
//		Eigen::Vector3f x_axis2,y_axis2,z_axis2;
//		std::vector<Eigen::Vector3f> axes2;
//		Eigen::Vector3f origin2;
//		std::vector<Eigen::Vector3f> contour2;
//		std::vector<std::vector<Eigen::Vector3f> > contours2;
//		Eigen::Vector3f v2;
//
//
//		x_axis2 << 0,-1,0;
//		axes2.push_back(x_axis2);
//
//		y_axis2 << 0,0,1;
//		axes2.push_back(y_axis2);
//
//		z_axis2 << 1,0,0;
//		axes2.push_back(z_axis2);
//
//		c2->axes_=axes2;
//
//		c2->r_ = 1;
//
//
//		v2 << 0, 1, 1;
//		contour2.push_back(v2);
//		v2 << 0, -1, 1;
//		contour2.push_back(v2);
//		v2 << 0 ,-1 ,-1;
//		contour2.push_back(v2);
//		v2 << 0, 1 ,-1;
//		contour2.push_back(v2);
//		contours2.push_back(contour2);
//
//
//		c2->merged=1;
//		origin2 << 0,0,0;
//		c2->origin_=origin2;
//
//
//		c2->holes.push_back(0);
//		c2->debug_=false;
//		c2->ContoursFromList(contours2);
//




		//transform cylinders

		float x,y,z,roll,pitch,yaw;

		x=1;
		y=2;
		z=3;
		roll=0.2;
		pitch=0.4;
		yaw=1;


		Eigen::Affine3f trafo;
		pcl::getTransformation(x,y,z,roll,pitch,yaw,trafo);

		transformCylinder(c1,trafo);
//		transformCylinder(c2,trafo);




		//		std::string s_c1 = "c1->unrolled";
		//		c1->debug_output(s_c1);
		//
		//		std::string s_c2 = "c2->unrolled";
		//		c2->debug_output(s_c2);





	}


	void
	publishShapes()
	{
		cob_3d_mapping_msgs::ShapeArray map_msg;
		map_msg.header.frame_id="/map";
		map_msg.header.stamp = ros::Time::now();
		cob_3d_mapping_msgs::Shape s;



//		PolygonPtr p1 = PolygonPtr(new Polygon());
//		map_msg.header.stamp = ros::Time::now();
//		makePolygon(p1);
//		toROSMsg(*p1,s);
//		s.header=map_msg.header;
//		s.color.r=0.5;
//		s.color.g=1;
//		s.color.a=1;
//		s.type=cob_3d_mapping_msgs::Shape::POLYGON;
//		map_msg.shapes.push_back(s);





		Cylinder::Ptr  c1  =Cylinder::Ptr(new Cylinder());
		makeCylinder(c1);
		toROSMsg(*c1, s);
		s.header = map_msg.header;
		s.color.b = 1;
		s.color.a = 1;
		s.type=cob_3d_mapping_msgs::Shape::CYLINDER;
		map_msg.shapes.push_back(s);

		map_pub_.publish(map_msg);


	}


	ros::NodeHandle n_;


protected:
	ros::Publisher map_pub_;


	GeometryMap geometry_map_;      /// map containing geometrys (polygons)



};

int main (int argc, char** argv)
{
	ros::init (argc, argv, "cylinder_client");

	cylinder_client cc;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		cc.publishShapes();
		ros::spinOnce ();
		loop_rate.sleep();
	}
}

//PLUGINLIB_DECLARE_CLASS(cob_env_model, FeatureMap, FeatureMap, nodelet::Nodelet)



