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
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <sensor_msgs/CameraInfo.h>

#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>

// Packages Includes:
#include "cob_3d_features/impl/invariant_surface_feature.hpp"
//#include "cob_3d_features/impl/invariant_surface_feature_unit_tests.hpp"
#include "cob_3d_features/impl/invariant_surface_feature_debug.hpp"
#include <cob_3d_segmentation/quad_regression/quad_regression.h>
#include <cob_3d_segmentation/quad_regression/polygon.h>

#include <cob_3d_mapping_common/stop_watch.h>
#include <cob_3d_visualization/simple_marker.h>

#include "test_invariant_feature_eval.hpp"

typedef PointXYZFeature64 FeaturePoint;
typedef PointXYZ SimplePoint;
	
class IFNode {
	typedef float Real;
	typedef double Scalar;
	typedef Sampler<Real, Scalar> S;
	
	enum {DEGREE=1};	/// degree of polynomial surfaces (input data)
	
	typedef Segmentation::S_POLYGON<DEGREE> Polygon;
	typedef cob_3d_features::InvariantSurfaceFeature< cob_3d_features::TriangleSurfaceHelper<cob_3d_mapping::Polygon> > ISF;
	//typedef cob_3d_features::InvariantSurfaceFeature< cob_3d_features::PolynomialSurfaceHelper<Polygon> > ISF;
	//typedef cob_3d_features::InvariantSurfaceFeature<Polygon> ISF;
	
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
		//isf_.addRadius(0.3);
		//isf_.addRadius(0.4);
		//isf_.addRadius(0.6);
		isf_.addRadius(0.8);
		
		n.param<std::string>("relative_frame_id", relative_frame_id_, "odom");
		
		exit(0);
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
	
	//typedef pcl::FPFHSignature33 Feature;
	typedef pcl::SHOT352 Feature;
	//typedef pcl::ESFSignature640 Feature;
	//typedef pcl::VFHSignature308 Feature;
		
	pcl::PointCloud<Feature>::Ptr fpfhs_old_;
	std::vector<Eigen::Vector3d> keypoints_old_;
	void cb_eval(const cob_3d_mapping_msgs::ShapeArrayConstPtr &msg, const sensor_msgs::PointCloud2ConstPtr &pc) {
		std::cout<<"evaluation cb"<<std::endl;
		
		sub_.shutdown();
		cb(*msg);
		
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg (*pc, *cloud);
		
		pcl::IndicesPtr indicies;
		pcl::PointCloud<pcl::PointNormal>::Ptr p_n2;
		computeNormals(cloud, isf_.getKeypoints(), indicies,p_n2);
		
		pcl::PointCloud<Feature>::Ptr fpfhs;
		computeFeatures<Feature>(indicies,p_n2, std::sqrt(isf_.getRadii()[0]), fpfhs);
		
		assert(fpfhs->size()==isf_.getKeypoints().size());
		
		if(fpfhs_old_ && fpfhs_old_->size()>0) {
			//std::cout<<"wait for key"<<std::endl;
			//getchar();
			
			pcl::search::KdTree<Feature>::Ptr tree (new pcl::search::KdTree<Feature> ());
			tree->setInputCloud(fpfhs_old_);
			for(size_t i=0; i<fpfhs->size(); i++) {
				if(!pcl::DefaultPointRepresentation<Feature>().isValid((*fpfhs)[i])) continue;
				
			  std::vector< int > k_indices;
			  std::vector< float > k_sqr_distances;
			  tree->nearestKSearch( (*fpfhs)[i], 1, k_indices, k_sqr_distances);
			  assert(k_indices.size()>0);
			  
			  int mii = k_indices[0];
			  
				cob_3d_visualization::RvizMarker arrow;
				Eigen::Vector3d o = keypoints_old_[mii]; o(2)+=2;
				arrow.arrow(isf_.getKeypoints()[i], o, 0.02);
				//double d = std::min((double)(keypoints_old_[mii]-isf_.getKeypoints()[i]).norm()+0.3, 1.);
				double d = (keypoints_old_[mii]-isf_.getKeypoints()[i]).norm()<std::sqrt(isf_.getRadii()[0])/2 ? 1.:0.3;
				arrow.color( d, d, 0. );
			}
		}
		
		fpfhs_old_ = fpfhs;
		keypoints_old_ = isf_.getKeypoints();
		cob_3d_visualization::RvizMarkerManager::get().publish();*/
	}
	
	void evaluateKeypoints(const cob_3d_mapping_msgs::ShapeArray &msg, const sensor_msgs::PointCloud2ConstPtr &pc, std::ostream &out, const std::vector<std::vector<double> > &parameters) {
		//cb_eval(boost::make_shared<cob_3d_mapping_msgs::ShapeArray>(msg), pc); //debugging
		
		PrecisionStopWatch sw;
		static const std::string DL="\t";
		static const std::string NL="\n";
				
		//write header (timestamp for tf lookup)
		out<<"header"<<DL<<pc->header.stamp;
		out<<NL;
		
		{ //our keypoint selection
			//input
			ISF::PTSurfaceList input;
			parseInput(msg, input);
	  
			//compute
			for(size_t i=0; parameters.size(); i++) {
				sw.precisionStart();
				//compute keypoints
				isf_.__evalKeypoints__setInput(input, parameters[i][0], parameters[i][1]);
				const double took1 = sw.precisionStop();
				
				std::string name = "FSHD"+DL+boost::lexical_cast<std::string>(parameters[i][0])+DL+boost::lexical_cast<std::string>(parameters[i][1]);
				
				//write duration
				out<<"took"<<DL<<name<<DL<<took1<<NL;
				
				//list of keypoints (ordered!)
				for(size_t i=0; i<isf_.getKeypoints().size(); i++)
					out<<"eval_keypoint"<<DL<<name<<DL<<isf_.getKeypoints()[i](0)<<DL<<isf_.getKeypoints()[i](1)<<DL<<isf_.getKeypoints()[i](2)<<NL;
			}
		}
		
		const static double colors[] = {0.3,1.,0.1, 0.1,1.,0.8};
		for(int keypoint_type=1; keypoint_type<=1; keypoint_type++) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromROSMsg (*pc, *cloud);
			pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
		
			const double took1 = detectKeypoints(keypoint_type, cloud, keypoints);
			
			std::string name = "Keypoint"+DL+boost::lexical_cast<std::string>(keypoint_type)+DL+"0";
			
			//write duration
			out<<"took"<<DL<<name<<DL<<took1<<NL;
			
			//list of keypoints (ordered!)
			for(size_t i=0; i<keypoints->size(); i++)
				out<<"eval_keypoint"<<DL<<name<<DL<<(*keypoints)[i].x<<DL<<(*keypoints)[i].y<<DL<<(*keypoints)[i].z<<NL;

			for(size_t i=0; i<keypoints->size(); i++)
			{
				cob_3d_visualization::RvizMarker scene;
				scene.sphere((*keypoints)[i].getVector3fMap(), 0.05);
				scene.color(colors[keypoint_type*3+0],colors[keypoint_type*3+1],colors[keypoint_type*3+2]);
			}
		}		
	}
	
	void evaluate(const cob_3d_mapping_msgs::ShapeArray &msg, const sensor_msgs::PointCloud2ConstPtr &pc, std::ostream &out, pcl::PointCloud<FeaturePoint> &pc_features, pcl::PointCloud<SimplePoint> &pc_kps) {
		//cb_eval(boost::make_shared<cob_3d_mapping_msgs::ShapeArray>(msg), pc); //debugging
		
		PrecisionStopWatch sw;
		static const std::string DL="\t";
		static const std::string NL="\n";
		
		{ //our descriptor
			//input
			ISF::PTSurfaceList input;
			parseInput(msg, input);
	  
			//compute
			sw.precisionStart();
			isf_.setInput(input);
			const double took1 = sw.precisionStop();
			
			//compute features
			sw.precisionStart();
			isf_.compute();
			const double took2 = sw.precisionStop();
			
			ROS_INFO("%d keypoints", (int)isf_.getKeypoints().size());
			if(isf_.getKeypoints().size()<1) return;	//no keypoints found...skipping
		
			//write header (timestamp for tf lookup)
			out<<"header"<<DL<<pc->header.stamp;
			for(size_t i=0; i<isf_.getRadii().size(); i++) out<<DL<<std::sqrt(isf_.getRadii()[i]);
			out<<NL;
			
			//write duration
			out<<"took"<<DL<<"FSHD"<<DL<<took1<<DL<<took2<<NL;
			std::cout<<"took"<<DL<<"FSHD"<<DL<<took1<<DL<<took2<<NL;
		}
		
		//list of keypoints (ordered!)
		for(size_t i=0; i<isf_.getKeypoints().size(); i++)
			out<<"keypoint"<<DL<<isf_.getKeypoints()[i](0)<<DL<<isf_.getKeypoints()[i](1)<<DL<<isf_.getKeypoints()[i](2)<<NL;
			
		//write out our results
		{
			ISF::PResultConst res = isf_.getResult();
			assert(isf_.getKeypoints().size()==res->size());
			for(size_t i=0; i<isf_.getKeypoints().size(); i++) {
				out<<"keypoint"<<DL<<"FSHD";
				std::vector<float> ftvec;
				(*res)[i].toVector(ftvec);
				for(size_t j=0; j<ftvec.size(); j++) out<<DL<<ftvec[j];
				out<<NL;
			}
			
			pc_kps.clear();
			for(size_t i=0; i<isf_.getAllKeypoints().size(); i++)
			{
				SimplePoint pt;
				pt.x = isf_.getAllKeypoints()[i](0);
				pt.y = isf_.getAllKeypoints()[i](1);
				pt.z = isf_.getAllKeypoints()[i](2);
				pc_kps.push_back(pt);
			}
		
			pc_features.clear();
			for(size_t i=0; i<res->size(); i++) {
				FeaturePoint pt;
				pt.x = (*res)[i].pt_(0);
				pt.y = (*res)[i].pt_(1);
				pt.z = (*res)[i].pt_(2);
				pt.area = (*res)[i].area_;
				assert(FeaturePoint::DIMENSION*2==(*res)[i].f_[0].values.size());
				for(size_t j=0; j<(*res)[i].f_[0].values.size(); j+=2)
					pt.feature[j/2] = (*res)[i].f_[0].values[j];
				pc_features.push_back(pt);
			}
			
		}
		
		return;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg (*pc, *cloud);
		
		pcl::IndicesPtr indicies;
		pcl::PointCloud<pcl::PointNormal>::Ptr p_n2;
		const double took_normal = computeNormals(cloud, isf_.getKeypoints(), indicies,p_n2);
		
		evalFeatures<pcl::FPFHSignature33>	(indicies, p_n2, std::sqrt(isf_.getRadii()[0]), out, took_normal, "FPFH");
		evalFeatures<pcl::SHOT352>			(indicies, p_n2, std::sqrt(isf_.getRadii()[0]), out, took_normal, "SHOT");
		evalFeatures<pcl::ESFSignature640>	(indicies, p_n2, std::sqrt(isf_.getRadii()[0]), out, took_normal, "ESF");
		evalFeatures<pcl::VFHSignature308>	(indicies, p_n2, std::sqrt(isf_.getRadii()[0]), out, took_normal, "VFH");
		
	}
	
	template<class FeatureT>
	double evalFeatures(pcl::IndicesPtr &indicies, pcl::PointCloud<pcl::PointNormal>::Ptr &p_n2, const float radius, std::ostream &out, const double took1, const char *FeatureName) {
		static const std::string DL="\t";
		static const std::string NL="\n";
		
		typename pcl::PointCloud<FeatureT>::Ptr fpfhs;
		const double r=computeFeatures<FeatureT>(indicies,p_n2, std::sqrt(isf_.getRadii()[0]), fpfhs);
		std::cout<<"NUM"<<fpfhs->size()<<std::endl;
		
		//write duration
		out<<"took"<<DL<<FeatureName<<DL<<took1<<DL<<r<<NL;
		
		//write out our results
		{
			assert(isf_.getKeypoints().size()==fpfhs->size());
			for(size_t i=0; i<isf_.getKeypoints().size(); i++) {
				out<<"keypoint"<<DL<<FeatureName;
				std::vector<float> ftvec;
				serialize_feature((*fpfhs)[i], ftvec);
				for(size_t j=0; j<ftvec.size(); j++) out<<DL<<ftvec[j];
				out<<NL;
			}
		}
		return r;
	}
	
	void parseInput(const cob_3d_mapping_msgs::ShapeArray &msg, ISF::PTSurfaceList &input) {
		//input
		input.reset(new ISF::TSurfaceList);
		for(size_t i=0; i<msg.shapes.size(); i++)
			input->push_back(ISF::TSurface::parse(msg.shapes[i]));
	}
	
	/// callback to shapes/surfaces --> input data of this feature
	void cb(const cob_3d_mapping_msgs::ShapeArray &msg) {
		/*static bool first=true;
		if(!first) {
			std::cout<<"wait for key"<<std::endl;
			getchar();
		}
		first=false;*/
			
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
				
#if 1
				cob_3d_visualization::RvizMarker arrow, txt2;
				Eigen::Vector3d o = (*last_)[mii].pt_; o(2)+=2;
				arrow.arrow((*oldR)[i].pt_, o, 0.02);
				mi = std::min((double)mi/BORDER, 1.);
				//arrow.color( std::min((double)((*oldR)[i].pt_-(*last_)[mii].pt_).norm(), 1.) ,mi,mi);
				arrow.color( ((*oldR)[i].pt_-(*last_)[mii].pt_).norm()<std::sqrt(isf_.getRadii()[0])/2?1:0.3 ,mi,mi);
				char buf[128];
				sprintf(buf, "%d", (int) i);
				txt2.text(buf);
				txt2.move( (*oldR)[i].pt_ );
				txt2.color(0,0,0);
#endif
#endif
			}
		}
		for(size_t i=0; last_ && i<last_->size(); i++) {
			cob_3d_visualization::RvizMarker txt;
			char buf[128];
			sprintf(buf, "%d", (int)i);
			txt.text(buf);
			txt.move( (*last_)[i].pt_+2*Eigen::Vector3d::UnitZ() );
			txt.color(0,0,0);
		}
		
		last_ = oldR;
		cob_3d_visualization::RvizMarkerManager::get().publish();
		
		std::cout<<"g_num_computations "<<g_num_computations<<std::endl;
	}
};

cob_3d_mapping_msgs::ShapeArray gl_shape_msg;
bool gl_shape_msg_set = false;
void static_eval_cb(const cob_3d_mapping_msgs::ShapeArray &msg) {
	gl_shape_msg = msg;
	gl_shape_msg_set=true;
	std::cout<<"got shapes"<<std::endl;
}

void evaluation(const std::string &fn, const std::string &ofn, const double radius, const int num_radii=8, const int num_angles=32, int skip=0, const bool eval_kp=false, const std::string &ofn_bag="") {	
	if(skip<1) skip=1;
	IFNode node(num_radii, num_angles, radius);
	
	std::vector<std::vector<double> > parameters;
	for(double min_area=0.005; min_area<0.05; min_area+=0.005) {
		parameters.push_back(std::vector<double>());
		for(double area=0.025; area<=0.15; area+=0.025) {
			parameters.back().push_back(min_area*min_area);
			parameters.back().push_back(area*area);
		}
	}
	
	std::cout<<"got "<<parameters.size()<<" parameters"<<std::endl;
	
	std::ofstream ofstr(ofn.c_str());
	
	ofstr<<"file\t"<<fn<<"\n";
	ofstr<<"radius\t"<<radius<<"\n";
	ofstr<<"num_radii\t"<<num_radii<<"\n";
	ofstr<<"num_angles\t"<<num_angles<<"\n";
	ofstr<<"skip\t"<<skip<<"\n";
	
	rosbag::Bag bag, bag_out;
    bag.open(fn, rosbag::bagmode::Read);
    if(ofn_bag.size()>0)
		bag_out.open(ofn_bag, rosbag::bagmode::Write);

	//here our only concern is the pointcloud
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/points"));
    topics.push_back(std::string("/camera/depth/camera_info"));
    topics.push_back(std::string("/tf"));
    
    //we use ros publish/subscriber to transform pc to shape
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/shapes_array", 1, &static_eval_cb);
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_xyzrgb", 1);
	
	//wait till everything is up and running
	ROS_INFO("waiting for segmentation");
	while(pub.getNumSubscribers()<1)
		sleep(1);
	ROS_INFO("segmentation found");

    rosbag::View view(bag, rosbag::TopicQuery(topics));

	rosbag::View::iterator view_it = view.begin();
	//first set FoV
    while(view_it!=view.end()&&ros::ok())
    {
        sensor_msgs::CameraInfo::ConstPtr ci = view_it->instantiate<sensor_msgs::CameraInfo>();
        if (ci != NULL) {
			std::cout<<"found camera info"<<std::endl;
			node.cb_camera_info(*ci);
			break;
		}
		view_it++;
	}
	assert(view_it!=view.end());
	
	std::cout<<"reading pointclouds"<<std::endl;
	view_it = view.begin();
	int num=0;
    while(view_it!=view.end()&&ros::ok())
    {
        sensor_msgs::PointCloud2::Ptr pc = view_it->instantiate<sensor_msgs::PointCloud2>();
        if (pc != NULL) {
			++num;
			if(num%skip==0) {
				//transform pc to shape msg
				gl_shape_msg_set = false;
				pc->header.frame_id = "/camera_rgb_optical_frame";
				pub.publish(pc);
				while(!gl_shape_msg_set) {
					ros::spinOnce();
					if(!ros::ok()) return;
				}
				
				//do evaluation
				if(eval_kp)
					node.evaluateKeypoints(gl_shape_msg, pc, ofstr, parameters);
				else {
					pcl::PointCloud<FeaturePoint> features;
					pcl::PointCloud<SimplePoint> keypoints;
					
					node.evaluate(gl_shape_msg, pc, ofstr, features, keypoints);
					
					if(ofn_bag.size()>0) {
						sensor_msgs::PointCloud2 features2;
						pcl::toROSMsg(features, features2);
						features2.header = gl_shape_msg.header;
						sensor_msgs::PointCloud2 keypoints2;
						pcl::toROSMsg(keypoints, keypoints2);
						keypoints2.header = gl_shape_msg.header;
						bag_out.write("/keypoints",view_it->getTime(), keypoints2);
						bag_out.write("/features", view_it->getTime(), features2);
					}
					
				}
				
				ofstr.flush();	//safety
			}
		}
		
		if(ofn_bag.size()>0 && view_it->getTopic()=="/tf") {
			bag_out.write(view_it->getTopic(), view_it->getTime(), *view_it, view_it->getConnectionHeader());
		}
		
		view_it++;
    }

    bag.close();
    ofstr.close();
    bag_out.close();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "invariant_feature");
	cob_3d_visualization::RvizMarkerManager::get().createTopic("/marker").setFrameId("/camera_rgb_optical_frame").clearOld();
	
	if(argc>1 && std::string(argv[2])=="--help") {
		std::cout<<"start either without arguments or with [bag file] [result file, csv] [radius]"<<std::endl;
		return 0;
	}
	else if(argc==4 && std::string(argv[1]).find(".bag")!=std::string::npos) {
		std::cout<<"Evaluation Mode 1"<<std::endl;
		evaluation(argv[1], argv[2], boost::lexical_cast<double>(argv[3]), 8, 32, 1);
		return 0;
	}
	else if(argc==5 && std::string(argv[1]).find(".bag")!=std::string::npos) {
		std::cout<<"Evaluation Mode 3"<<std::endl;
		evaluation(argv[1], argv[2], boost::lexical_cast<double>(argv[3]), 8, 32, 1, false, argv[4]);
		return 0;
	}
	else if(argc==6 && std::string(argv[1]).find(".bag")!=std::string::npos) {
		std::cout<<"Evaluation Mode 2"<<std::endl;
		evaluation(argv[1], argv[2], boost::lexical_cast<double>(argv[3]), boost::lexical_cast<int>(argv[4]), boost::lexical_cast<int>(argv[5]), 1);
		return 0;
	}
	
#ifndef EIGEN_VECTORIZE
	ROS_INFO("this would be faster if SIMD is enabled");
#endif

	int num_radii = 8;
	int num_angles = 32;
	
	IFNode node(num_radii, num_angles);
	
	ros::spin();
}
