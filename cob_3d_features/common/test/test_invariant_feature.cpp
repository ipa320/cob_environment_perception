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
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
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
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <Eigen/Geometry>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <sensor_msgs/CameraInfo.h>

// Packages Includes:
#include "cob_3d_features/impl/invariant_surface_feature.hpp"
//#include "cob_3d_features/impl/invariant_surface_feature_unit_tests.hpp"
#include "cob_3d_features/impl/invariant_surface_feature_debug.hpp"
#include <cob_3d_segmentation/quad_regression/quad_regression.h>
#include "cob_3d_segmentation/quad_regression/polygon.h"

#include <cob_3d_mapping_common/stop_watch.h>
#include <simple_marker.h>

#include "test_invariant_feature_eval.hpp"

class IFNode {
	typedef float Real;
	typedef double Scalar;
	typedef Sampler<Real, Scalar> S;
	
	enum {DEGREE=2};	/// degree of polynomial surfaces (input data)
	
	typedef Segmentation::S_POLYGON<DEGREE> Polygon;
	typedef cob_3d_features::InvariantSurfaceFeature<Polygon> ISF;
	
	ISF isf_;	/// computation unit
	//boost::shared_ptr<ISF> isf_;
	
	//ros stuff
	ros::Subscriber sub_, sub_camera_info_;
	tf::TransformListener tf_listener_;
	typedef message_filters::sync_policies::ApproximateTime<cob_3d_mapping_msgs::ShapeArray, sensor_msgs::PointCloud2> MySyncPolicy;
	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_;
	//boost::shared_ptr<message_filters::TimeSynchronizer<cob_3d_mapping_msgs::ShapeArray, sensor_msgs::PointCloud2> > sync_;
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
	
public:
	IFNode(const int num_radii, const int num_angles):
		isf_(num_radii, num_angles)
	{
		ros::NodeHandle n;
		
		sub_ = n.subscribe("/shapes_array", 1, &IFNode::cb, this);
		
		//alternative to sub_ for evaluation mode
		message_filters::Subscriber<cob_3d_mapping_msgs::ShapeArray> *sa_sub = new message_filters::Subscriber<cob_3d_mapping_msgs::ShapeArray>(n, "shapes_array", 1);
		message_filters::Subscriber<sensor_msgs::PointCloud2> *pc_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(n, "camera/depth/points_xyzrgb", 1);

		sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *sa_sub, *pc_sub) );
		//sync_.reset( new message_filters::TimeSynchronizer<cob_3d_mapping_msgs::ShapeArray, sensor_msgs::PointCloud2>(sa_sub, pc_sub, 5) );
		sync_->registerCallback(boost::bind(&IFNode::cb_eval, this, _1, _2));

		sub_camera_info_ = n.subscribe("/camera/depth/camera_info", 0, &IFNode::cb_camera_info, this);
		
		//parameters
		
		//TODO: make configurable
		isf_.addRadius(0.3);
		isf_.addRadius(0.6);
		
		n.param<std::string>("relative_frame_id", relative_frame_id_, "odom");
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
	}
	
	//typedef pcl::FPFHSignature33 Feature;
	//typedef pcl::SHOT352 Feature;
	typedef pcl::ESFSignature640 Feature;
	//typedef pcl::VFHSignature308 Feature;
		
	pcl::PointCloud<Feature>::Ptr fpfhs_old_;
	std::vector<Eigen::Vector3d> keypoints_old_;
	void cb_eval(const cob_3d_mapping_msgs::ShapeArrayConstPtr &msg, const sensor_msgs::PointCloud2ConstPtr &pc) {
		std::cout<<"evaluation cb"<<std::endl;
		
		sub_.shutdown();
		cb(*msg);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg (*pc, *cloud);
		
		pcl::PointCloud<Feature>::Ptr fpfhs;
		computeFeatures<Feature>(cloud, isf_.getRadii()[0], fpfhs, isf_.getKeypoints());
		
		assert(fpfhs->size()==isf_.getKeypoints().size());
		
		if(fpfhs_old_) {
			pcl::search::KdTree<Feature>::Ptr tree (new pcl::search::KdTree<Feature> ());
			tree->setInputCloud(fpfhs_old_);
			for(size_t i=0; i<fpfhs->size(); i++) {
			  std::vector< int > k_indices;
			  std::vector< float > k_sqr_distances;
			  tree->nearestKSearch( (*fpfhs)[i], 1, k_indices, k_sqr_distances);
			  assert(k_indices.size()>0);
			  
			  int mii = k_indices[0];
			  
				cob_3d_visualization::RvizMarker arrow;
				Eigen::Vector3d o = keypoints_old_[mii]; o(2)+=2;
				arrow.arrow(isf_.getKeypoints()[i], o, 0.02);
				double d = std::min((double)(keypoints_old_[mii]-isf_.getKeypoints()[i]).norm()+0.3, 1.);
				arrow.color( d, d, 0. );
			}
		}
		
		fpfhs_old_ = fpfhs;
		keypoints_old_ = isf_.getKeypoints();
		cob_3d_visualization::RvizMarkerManager::get().publish();
	}
	
	/// callback to shapes/surfaces --> input data of this feature
	void cb(const cob_3d_mapping_msgs::ShapeArray &msg) {
		static bool first=true;
		if(!first) {
			std::cout<<"wait for key"<<std::endl;
			getchar();
		}
		first=false;
			
		//input
		size_t dbg_points = 0;
		ISF::PTSurfaceList input(new ISF::TSurfaceList);
		for(size_t i=0; i<msg.shapes.size(); i++) {
			Polygon p;
			assert((int)msg.shapes[i].params.size() <= (int)p.model_.p.size());
			for(size_t j=0; j<msg.shapes[i].params.size(); j++)
				p.model_.p(j) = msg.shapes[i].params[j];
				
			p.segments_.resize(msg.shapes[i].points.size());
			for(size_t j=0; j<msg.shapes[i].points.size(); j++) {
				//std::cout<<j<<" "<<(j!=0)<<" "<<(bool)msg.shapes[i].holes[j]<<std::endl;
				//assert( (j!=0)==(bool)msg.shapes[i].holes[j] );
				
				pcl::PointCloud<pcl::PointXYZ> pc;
				pcl::fromROSMsg(msg.shapes[i].points[j], pc);
				dbg_points += pc.size();

				for(size_t k=0; k<pc.size(); k++)
					p.segments_[j].push_back(pc[k].getVector3fMap());
			}
			input->push_back(p);
		}
		std::cout<<"read data "<<dbg_points<<std::endl;
		
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
		
		ISF::PResultConst oldR = isf_.getResult();
		std::cout<<"got result"<<std::endl;
		
		cob_3d_visualization::RvizMarkerManager::get().clear();
		if(old_scene_.polygons.size()>0) {
			cob_3d_visualization::RvizMarker scene;
			scene.mesh(old_scene_);
			scene.move(0, 0, 2.);
			scene.color(1.,0.9,0.9,0.9);
		}
		{
			cob_3d_visualization::RvizMarker scene;
			scene.mesh(*isf_.dbg_Mesh_of_Map());
		}
		old_scene_ = *isf_.dbg_Mesh_of_Map();
		
		for(size_t i=0; i<isf_.getKeypoints().size(); i++)
		{
			cob_3d_visualization::RvizMarker scene;
			scene.sphere(isf_.getKeypoints()[i], 0.05);
			scene.color(0.1,1.,0.1);
		}
		
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
		const Real BORDER = 1000;
		if(last_) {
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
				
				const float A = ((*oldR)[i].area_+(*last_)[mii].area_)/2;
				std::cout<<"min: "<<mi<<"/"<<mm/last_->size()<<"/"<<ma<<" "<<((*oldR)[i].pt_-(*last_)[mii].pt_).norm()<<" "<<i<<" "<<mii<<std::endl;
				std::cout<<"\tmin: "<<mi/A<<"/"<<mm/last_->size()/A<<"/"<<ma/A<<std::endl;
				if(mi/A>BORDER) continue;
				
#if 1
				cob_3d_visualization::RvizMarker arrow, txt;
				Eigen::Vector3d o = (*last_)[mii].pt_; o(2)+=2;
				arrow.arrow((*oldR)[i].pt_, o, 0.02);
				mi = std::min((double)mi/BORDER, 1.);
				arrow.color( std::min((double)((*oldR)[i].pt_-(*last_)[mii].pt_).norm(), 1.) ,mi,mi);
				char buf[128];
				sprintf(buf, "%d -> %d", (int) i, (int)mii);
				txt.text(buf);
				txt.move( ((*oldR)[i].pt_+o)/2 );
#endif
#endif
			}
		}
		
		last_ = oldR;
		cob_3d_visualization::RvizMarkerManager::get().publish();
		
		std::cout<<"g_num_computations "<<g_num_computations<<std::endl;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "invariant_feature");
	cob_3d_visualization::RvizMarkerManager::get().createTopic("/marker").setFrameId("/camera_rgb_optical_frame").clearOld();
	
#ifndef EIGEN_VECTORIZE
	ROS_INFO("this would be faster if SIMD is enabled");
#endif

	int num_radii = 8;
	int num_angles = 32;
	
	IFNode node(num_radii, num_angles);
	
	ros::spin();
}
