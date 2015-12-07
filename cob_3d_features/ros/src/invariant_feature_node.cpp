/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: Joshua Hampp
 *
 * \date Date of creation: 06/01/2014
 *
 * \brief
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

int g_num_computations;

#include <fstream>
#include <iostream>

//ros
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <sensor_msgs/CameraInfo.h>

// Packages Includes:
#include <cob_3d_mapping_common/point_types.h>
#include "cob_3d_features/impl/invariant_surface_feature.hpp"
//#include "cob_3d_features/impl/invariant_surface_feature_unit_tests.hpp"
#include "cob_3d_features/impl/invariant_surface_feature_debug.hpp"
//#include <cob_3d_segmentation/quad_regression/quad_regression.h>
#include "../../../cob_3d_segmentation/common/include/cob_3d_segmentation/quad_regression/polygon.h"
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include <cob_3d_mapping_common/point_types.h>

#include <cob_3d_mapping_common/stop_watch.h>
#include <cob_3d_visualization/simple_marker.h>

//#include "test_invariant_feature_eval.hpp"

class IFNode {
	typedef float Real;
	typedef double Scalar;
	typedef Sampler<Real, Scalar> S;
	typedef PointXYZFeature64 FeaturePoint;
	typedef PointXYZ SimplePoint;
	
	enum {DEGREE=1};	/// degree of polynomial surfaces (input data)
	
	typedef cob_3d_features::InvariantSurfaceFeature< cob_3d_features::TriangleSurfaceHelper<cob_3d_mapping::Polygon> > ISF;
	//typedef Segmentation::S_POLYGON<DEGREE> Polygon;
	//typedef cob_3d_features::InvariantSurfaceFeature< cob_3d_features::PolynomialSurfaceHelper<Polygon> > ISF;
	
	ISF isf_;	/// computation unit
	//boost::shared_ptr<ISF> isf_;
	
	//ros stuff
	ros::Subscriber sub_, sub_camera_info_;
	ros::Publisher  pub_features_, pub_keypoints_;
	tf::TransformListener tf_listener_;
	std::string relative_frame_id_;
	
	ISF::PResultConst last_; /// features of last frame
	pcl::PolygonMesh old_scene_; /// debug map of last frame
	std::vector<Eigen::Vector4f> fov_; /// field of view (compatible to computation unit)
	
	bool getTransformation(const ros::Time &time, const std::string &frame_id, typename ISF::TAffine &tf) {
		tf::StampedTransform transform;
		try{
		  tf_listener_.lookupTransform(relative_frame_id_, frame_id,
								   time, transform);
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  return false;
		}
		
		Eigen::Quaternion<typename ISF::TAffine::Scalar> q;
		q.x() = transform.getRotation().x();
		q.y() = transform.getRotation().y();
		q.z() = transform.getRotation().z();
		q.w() = transform.getRotation().w();
		Eigen::Translation<typename ISF::TAffine::Scalar, 3> t;
		t.x() = transform.getOrigin().x();
		t.y() = transform.getOrigin().y();
		t.z() = transform.getOrigin().z();
		tf = q*t;
		
		return true;
	}
	
	void parseInput(const cob_3d_mapping_msgs::ShapeArray &msg, ISF::PTSurfaceList &input) {
		//input
		input.reset(new ISF::TSurfaceList);
		for(size_t i=0; i<msg.shapes.size(); i++)
			input->push_back(ISF::TSurface::parse(msg.shapes[i]));
	}
	
public:
	IFNode(const int num_radii, const int num_angles):
		isf_(num_radii, num_angles)
	{
		ros::NodeHandle n;
		
		pub_features_ = n.advertise<sensor_msgs::PointCloud2>("features", 1);
		pub_keypoints_= n.advertise<sensor_msgs::PointCloud2>("keypoints", 1);
		sub_ = n.subscribe("/shapes_array", 1, &IFNode::cb, this);
		sub_camera_info_ = n.subscribe("/camera/depth/camera_info", 0, &IFNode::cb_camera_info, this);
		
		//parameters
		
		ros::NodeHandle pn("~");
		double radius;
		pn.param<double>("radius", radius, 0.9);
		//TODO: make configurable
		//isf_.addRadius(0.3);
		//isf_.addRadius(0.4);
		//isf_.addRadius(0.6);
		isf_.addRadius(radius);
		
		n.param<std::string>("relative_frame_id", relative_frame_id_, "odom");
		
		ROS_INFO("using radius %f", radius);
	}
	
	//only for evaluation!
	IFNode(const int num_radii, const int num_angles, const double radius):
		isf_(num_radii, num_angles)
	{
		//TODO: set FoV!!!
		
		isf_.addRadius(radius);
	}
	
	/// the field of view is received ONCE, after that this method will not be called any more
	void cb_camera_info(const sensor_msgs::CameraInfo &ci) {
		sub_camera_info_.shutdown();
		
		//calc. field of view (FoV)
		fov_.clear();
		
		Eigen::Vector4f v;
		Eigen::Vector3f o,a,b,c,d; //o is camera origin, rest are edges of VoF
		
		o = Eigen::Vector3f::Zero();
		a=b=c=d = Eigen::Vector3f::UnitZ();
		
		a(0)=d(0) =  -1/ci.K[0]*ci.width/2;
		b(0)=c(0) =   1/ci.K[0]*ci.width/2;
		c(1)=d(1) =  -1/ci.K[4]*ci.height/2;
		a(1)=b(1) =   1/ci.K[4]*ci.height/2;
		
		//normals looking to the outside
		v.head<3>() = (a-o).cross(b-o).normalized();
		v(3) = v.head<3>().dot(o);
		fov_.push_back(v);
		
		v.head<3>() = (b-o).cross(c-o).normalized();
		v(3) = v.head<3>().dot(o);
		fov_.push_back(v);
		
		v.head<3>() = (c-o).cross(d-o).normalized();
		v(3) = v.head<3>().dot(o);
		fov_.push_back(v);
		
		v.head<3>() = (d-o).cross(a-o).normalized();
		v(3) = v.head<3>().dot(o);
		fov_.push_back(v);
		
		//set FoV to computation unit
		isf_.setFoV(fov_);
		
		for(size_t i=0; i<fov_.size(); i++)
			std::cout<<"fov "<<fov_[i].transpose()<<std::endl;
	}
	
	/// callback to shapes/surfaces --> input data of this feature
	void cb(const cob_3d_mapping_msgs::ShapeArray &msg) {
		static bool first=true;
		/*if(!first) {
			std::cout<<"wait for key"<<std::endl;
			getchar();
		}*/
		first=false;
			
		//input
		ISF::PTSurfaceList input;
		parseInput(msg, input);
		std::cout<<"read data "<<std::endl;
		
		PrecisionStopWatch sw;
  
		//compute
		sw.precisionStart();
		isf_.setInput(input);
		std::cout<<"setInput "<<sw.precisionStop()<<"s"<<std::endl;
		
		//debug
		pcl::io::savePLYFile("map1.ply", 		*isf_.dbg_Mesh_of_Map());
		
		//compute features
		sw.precisionStart();
		isf_.compute();
		std::cout<<"compute "<<sw.precisionStop()<<"s"<<std::endl;
			
		isf_.dbg_writeVolumeImage("volume.json");
		
		ISF::PResultConst oldR = isf_.getResult();
		std::cout<<"got result"<<std::endl;
		
		cob_3d_visualization::RvizMarkerManager::get().clear();
		/*if(old_scene_.polygons.size()>0) {
			cob_3d_visualization::RvizMarker scene;
			scene.mesh(old_scene_);
			scene.move(0, 0, 2.);
			scene.color(1.,0.9,0.9,0.9);
			scene.color(0.1,0.1,0.98);
		}
		{
			cob_3d_visualization::RvizMarker scene;
			scene.mesh(*isf_.dbg_Mesh_of_Map(),1.,1.,1.);
			scene.color(0.98,0.1,0.1);
		}*/
		old_scene_ = *isf_.dbg_Mesh_of_Map();
		
		for(size_t i=0; i<isf_.getKeypoints().size(); i++)
		{
			cob_3d_visualization::RvizMarker scene;
			scene.sphere(isf_.getKeypoints()[i], 0.05);
			scene.color(0.1,1.,0.1);
		}
		
		//publish features
		pcl::PointCloud<FeaturePoint> features;
		for(size_t i=0; i<oldR->size(); i++) {
			FeaturePoint pt;
			pt.x = (*oldR)[i].pt_(0);
			pt.y = (*oldR)[i].pt_(1);
			pt.z = (*oldR)[i].pt_(2);
			assert(FeaturePoint::DIMENSION*2==(*oldR)[i].f_[0].values.size());
			for(size_t j=0; j<(*oldR)[i].f_[0].values.size(); j+=2)
				pt.feature[j/2] = (*oldR)[i].f_[0].values[j];
			features.push_back(pt);
		}
		sensor_msgs::PointCloud2 features2;
		pcl::toROSMsg(features, features2);
		features2.header = msg.header;
		pub_features_.publish(features2);
		
		pcl::PointCloud<SimplePoint> keypoints;
		for(size_t i=0; i<isf_.getAllKeypoints().size(); i++)
		{
			SimplePoint pt;
			pt.x = isf_.getAllKeypoints()[i](0);
			pt.y = isf_.getAllKeypoints()[i](1);
			pt.z = isf_.getAllKeypoints()[i](2);
			keypoints.push_back(pt);
		}
		sensor_msgs::PointCloud2 keypoints2;
		pcl::toROSMsg(keypoints, keypoints2);
		keypoints2.header = msg.header;
		pub_keypoints_.publish(keypoints2);
		
		static int dbg_stage=0;
		++dbg_stage;
	
		for(size_t i=0; i<oldR->size(); i++) {
			for(size_t j=0; j<(*oldR)[i].f_.size(); j++) {
			  char dbg_fn[128];
			  sprintf(dbg_fn,"/tmp/ft%d_%d_%d.sig", (int)i, (int)j, dbg_stage);
			  (*oldR)[i].f_[j].write(dbg_fn, false);
			}
		}
		
		//find next neighbour
		const Real BORDER = 100;
		if(last_ && last_->size()>0 && oldR->size()>0) {
			for(size_t i=0; i<oldR->size(); i++) {
				Real mi=std::numeric_limits<Real>::max(), ma=0, mm=0;
				size_t mii=0;
#if 0
				mii=last_->size()/2;
				mi = ma = (*oldR)[i].distance((*last_)[mii]);
				std::cout<<"min: "<<mi<<"/"<<ma<<" "<<((*oldR)[i].pt_-(*last_)[mii].pt_).norm()<<" "<<i<<" "<<mii<<std::endl;
				
				cob_3d_visualization::RvizMarker arrow;
				Eigen::Vector3d o = (*last_)[mii].pt_; o(2)+=2;
				arrow.arrow((*oldR)[i].pt_, o, 0.02);
				mi = std::min((double)(mi)/(200.), 1.);
				arrow.color(mi,mi,mi);
#else
				for(size_t j=0; j<last_->size(); j++) {
					const Real d = (*oldR)[i].distance((*last_)[j]);
					if(d<mi) {
						mi=d;
						mii=j;
					}
					ma = std::max(d, ma);
					mm+= d;
#if 0
					const float A = ((*oldR)[i].area_+(*last_)[j].area_)/2;
					if(d/A<BORDER) {
						cob_3d_visualization::RvizMarker arrow, txt;
						Eigen::Vector3d o = (*last_)[j].pt_; o(2)+=2;
						arrow.arrow((*oldR)[i].pt_, o, 0.02);
						double temp = std::min((double)d/BORDER, 1.);
						arrow.color( std::min((double)((*oldR)[i].pt_-(*last_)[j].pt_).norm(), 1.) ,temp,temp);
					}
#endif
				}
				
				if(mii>=last_->size()) continue;
				
				const float A = ((*oldR)[i].area_+(*last_)[mii].area_)/2;
				std::cout<<"min: "<<mi<<"/"<<mm/last_->size()<<"/"<<ma<<" "<<((*oldR)[i].pt_-(*last_)[mii].pt_).norm()<<" "<<i<<" "<<mii<<std::endl;
				std::cout<<"\tmin: "<<mi/A<<"/"<<mm/last_->size()/A<<"/"<<ma/A<<std::endl;
				
				//if(mi/A>BORDER) continue;
	/*			
#if 1
				cob_3d_visualization::RvizMarker arrow;
				Eigen::Vector3d o = (*last_)[mii].pt_; o(2)+=2;
				arrow.arrow((*oldR)[i].pt_, o, 0.02);
				mi = std::min((double)mi/BORDER, 1.);
				//arrow.color( std::min((double)((*oldR)[i].pt_-(*last_)[mii].pt_).norm(), 1.) ,mi,mi);
				arrow.color(mi, ((*oldR)[i].pt_-(*last_)[mii].pt_).norm()<std::sqrt(isf_.getRadii()[0])/2?1:0.3 ,mi);
#else
				cob_3d_visualization::RvizMarker arrow, txt2;
				Eigen::Vector3d o = (*last_)[mii].pt_; o(2)+=2;
				arrow.arrow((*oldR)[i].pt_, o, 0.02);
				mi = std::min((double)mi/BORDER, 1.);
				//arrow.color( std::min((double)((*oldR)[i].pt_-(*last_)[mii].pt_).norm(), 1.) ,mi,mi);
				arrow.color(mi, ((*oldR)[i].pt_-(*last_)[mii].pt_).norm()<std::sqrt(isf_.getRadii()[0])/2?1:0.3 ,mi);
				char buf[128];
				sprintf(buf, "%d", (int) i);
				txt2.text(buf);
				txt2.move( (*oldR)[i].pt_ );
				txt2.color(0,0,0);
#endif*/
#endif
			}
		}
		/*for(size_t i=0; last_ && i<last_->size(); i++) {
			cob_3d_visualization::RvizMarker txt;
			char buf[128];
			sprintf(buf, "%d", (int)i);
			txt.text(buf);
			txt.move( (*last_)[i].pt_+2*Eigen::Vector3d::UnitZ() );
			txt.color(0,0,0);
		}*/
		
		last_ = oldR;
		cob_3d_visualization::RvizMarkerManager::get().publish();
		
		std::cout<<"g_num_computations "<<g_num_computations<<std::endl;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "invariant_feature_node");
	cob_3d_visualization::RvizMarkerManager::get().createTopic("invariant_feature_debug_markers").setFrameId("/camera_depth_optical_frame").clearOld();
	
#ifndef EIGEN_VECTORIZE
	ROS_INFO("this would be faster if SIMD is enabled");
#endif

	int num_radii = 8;
	int num_angles = 32;
	
	IFNode node(num_radii, num_angles);
	
	ros::spin();
}
