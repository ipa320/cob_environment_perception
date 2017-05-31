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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_visualization
 *
 * \author
 *  Author: Joshua Hampp, joshua.hampp@ipa.fraunhofer.de
 *
 * \date Date of creation: 09/2014
 *
 * \brief
 *
 *
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

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <list>
#include <string>
#include <Eigen/Geometry>

/* 
 * EXAMPLE USAGE:
 * 
 * once at startup for init.:
 * cob_3d_visualization::RvizMarkerManager::get().createTopic("/marker").setFrameId("/map").clearOld();
 * 
 * 1. creates a MarkerArray topic on /marker with the default frame id /map
 * 2. clears old markers
 * 
 * 
 * to visualize markers:
 * cob_3d_visualization::RvizMarkerManager::get().clear();
 * {
 * 	cob_3d_visualization::RvizMarker scene;
 * 	scene.sphere(center, 0.05);
 * 	scene.color(0.1,1.,0.1);
 * }
 * cob_3d_visualization::RvizMarkerManager::get().publish();
 * 
 * 1. clears markers
 * 2. creates a sphere marker with radius 0.05 metres at point "center"
 * 3. sets color
 * 4. publish all new created markers
 */

namespace cob_3d_visualization {
	
	class RvizMarkerManager {
		struct S_ACTIVE {
			int id;
			std::string ns;
			
			S_ACTIVE() {}
			S_ACTIVE(const int id, const std::string &ns) : id(id), ns(ns) {}
		};
		
		std::string ns_, frame_id_;
		int id_;
		std::list<S_ACTIVE> active_ids_;
		ros::Publisher vis_pub_;
		visualization_msgs::MarkerArray markers_;
		
		RvizMarkerManager(const std::string &ns) : ns_(ns), id_(1)
		{}
		
	public:
	
		static RvizMarkerManager &get() {
			static RvizMarkerManager manager("simple_markers");
			return manager;
		}
		
		RvizMarkerManager &setFrameId(const std::string &frame_id) {frame_id_=frame_id;return *this;}
		const std::string &frame_id() const {return frame_id_;}
		const std::string &ns() const {return ns_;}
		int newId() {return id_++;}
		
		RvizMarkerManager &createTopic(const std::string &topic_name) {
			ros::NodeHandle nh;
			vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>( topic_name, 0 );
			return *this;
		}
		void prepare(const visualization_msgs::Marker &marker) {markers_.markers.push_back(marker);}
		void publish(const ros::Time &stamp=ros::Time(), const bool keep = false) {
			for(size_t i=0; i<markers_.markers.size(); i++) {
				markers_.markers[i].header.stamp = stamp;
				if(markers_.markers[i].action!=visualization_msgs::Marker::DELETE)
					active_ids_.push_back(S_ACTIVE(markers_.markers[i].id, markers_.markers[i].ns)); 
			}
			vis_pub_.publish(markers_);
			if(!keep) markers_.markers.clear();
			ros::NodeHandle nh;
			nh.setParam("/simple_marker/"+ns()+"/max", id_);
		}
		
		void clearOld() {
			ros::NodeHandle nh;
			int mi, ma;
			nh.param<int>("/simple_marker/"+ns()+"/min", mi, 0);
			nh.param<int>("/simple_marker/"+ns()+"/max", ma, 0);
			ROS_INFO("clearing old markers from %d to %d", mi, ma);
			for(; mi<ma; mi++)
				active_ids_.push_back(S_ACTIVE(mi,""));
				
			ros::Rate poll_rate(20);
			int num = 0;
			while(vis_pub_.getNumSubscribers() == 0 && (num++)<100)
				poll_rate.sleep();
				
			if(vis_pub_.getNumSubscribers() == 0)
				ROS_WARN("could not clear markers, as no subscribers are present at the moment");
		
			clear();
		}
		
		bool needed() const {return vis_pub_.getNumSubscribers()>0;}
		
		void clear(const ros::Time &stamp=ros::Time()) {
			if(active_ids_.size()>0) {
				ros::NodeHandle nh;
				nh.setParam("/simple_marker/"+ns()+"/min", active_ids_.back().id);
			}
			
			while(active_ids_.size()>0) {
				visualization_msgs::Marker marker;
				marker.header.frame_id = frame_id();
				marker.header.stamp = stamp;
				marker.ns = active_ids_.back().ns;
				marker.id = active_ids_.back().id;
				marker.action = visualization_msgs::Marker::DELETE;
				
				active_ids_.pop_back();
				prepare(marker);
			}
			
			publish(stamp);
		}
	};
	
	class RvizMarker {
	protected:
		void setDefaults(const ros::Time &time) {
			marker_.header.frame_id = RvizMarkerManager::get().frame_id();
			marker_.header.stamp = time;
			marker_.ns = RvizMarkerManager::get().ns();
			marker_.id = RvizMarkerManager::get().newId();
			marker_.type = visualization_msgs::Marker::SPHERE;
			marker_.action = visualization_msgs::Marker::ADD;
			marker_.pose.position.x = 0;
			marker_.pose.position.y = 0;
			marker_.pose.position.z = 0;
			marker_.pose.orientation.x = 0.0;
			marker_.pose.orientation.y = 0.0;
			marker_.pose.orientation.z = 0.0;
			marker_.pose.orientation.w = 1.0;
			marker_.scale.x = 1;
			marker_.scale.y = 1;
			marker_.scale.z = 1;
			marker_.color.a = 1.0;
			marker_.color.r = 1.0;
			marker_.color.g = 1.0;
			marker_.color.b = 1.0;
		}
		
		visualization_msgs::Marker marker_;
		
		template<class Vector>
		void primitive(const Vector &center, const float scale, int type) {
			marker_.type = type;
			marker_.pose.position = _2geometry(center);
			marker_.scale.x = marker_.scale.y = marker_.scale.z = scale;
		}
		
	public:
	
		RvizMarker() {
			setDefaults(ros::Time());
		}
		
		~RvizMarker() {RvizMarkerManager::get().prepare(marker_);}
		
		operator visualization_msgs::Marker() const {return marker_;}
		
		template<class Vector>
		static inline geometry_msgs::Point _2geometry(const Vector &v) {
			geometry_msgs::Point pt;
			pt.x=v(0); pt.y=v(1); pt.z=v(2);
			return pt;
		}
		
		template<class Vector>
		RvizMarker &addTriangle(const Vector &v1, const Vector &v2, const Vector &v3) {
			marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
			marker_.points.push_back(_2geometry(v1));
			marker_.points.push_back(_2geometry(v2));
			marker_.points.push_back(_2geometry(v3));
			return *this;
		}
		
#ifdef PCL_VERSION
		void mesh(const pcl::PolygonMesh &mesh) {
			pcl::PointCloud<pcl::PointXYZ> points;
			//pcl::fromROSMsg(mesh.cloud, points);
			pcl::fromPCLPointCloud2(mesh.cloud, points);
			for(size_t i=0; i<mesh.polygons.size(); i++)
				for(int j=0; j<(int)mesh.polygons[i].vertices.size()-2; j++) {
					addTriangle(
						points[mesh.polygons[i].vertices[j  ]].getVector3fMap(),
						points[mesh.polygons[i].vertices[j+1]].getVector3fMap(),
						points[mesh.polygons[i].vertices[j+2]].getVector3fMap()
					);
			}
		}
		
		void mesh(const pcl::PolygonMesh &mesh, const double r, const double g,  const double b, const double a=1., const Eigen::Vector3f dir=Eigen::Vector3f::UnitZ(), const float amb=0.2f) {
			pcl::PointCloud<pcl::PointXYZ> points;
			//pcl::fromROSMsg(mesh.cloud, points);
			pcl::fromPCLPointCloud2(mesh.cloud, points);
			for(size_t i=0; i<mesh.polygons.size(); i++)
				for(int j=0; j<(int)mesh.polygons[i].vertices.size()-2; j++) {
					addTriangle(
						points[mesh.polygons[i].vertices[j  ]].getVector3fMap(),
						points[mesh.polygons[i].vertices[j+1]].getVector3fMap(),
						points[mesh.polygons[i].vertices[j+2]].getVector3fMap()
					);
					
					Eigen::Vector3f normal = (points[mesh.polygons[i].vertices[j  ]].getVector3fMap()-points[mesh.polygons[i].vertices[j+1]].getVector3fMap())
					.cross(points[mesh.polygons[i].vertices[j+2]].getVector3fMap()-points[mesh.polygons[i].vertices[j+1]].getVector3fMap()).normalized();
					
					float F = std::abs(normal.dot(dir))*(1-amb)+amb;
					
					std_msgs::ColorRGBA c;
					c.a = a;
					c.r = r*F;
					c.g = g*F;
					c.b = b*F;
					marker_.colors.push_back(c);
					marker_.colors.push_back(c);
					marker_.colors.push_back(c);
			}
		}
#endif
		
		template<class Vector>
		void arrow(const Vector &start, const Vector &end, const float scale=0) {
			marker_.type = visualization_msgs::Marker::ARROW;
			
			marker_.points.clear();
			marker_.points.push_back(_2geometry(start));
			marker_.points.push_back(_2geometry(end));
			
			geometry_msgs::Point delta;
			delta.x = marker_.points[0].x-marker_.points[1].x;
			delta.y = marker_.points[0].y-marker_.points[1].y;
			delta.z = marker_.points[0].z-marker_.points[1].z;
			const float l = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
			
			marker_.scale.x = (scale?scale:0.1*l);
			marker_.scale.y = 2*marker_.scale.x;
			marker_.scale.z = 0;
		}
		
		template<class Vector>
		void line(const Vector &start, const Vector &end, float scale=0) {
			marker_.type = visualization_msgs::Marker::LINE_STRIP;
			
			marker_.points.clear();
			marker_.points.push_back(_2geometry(start));
			marker_.points.push_back(_2geometry(end));
			
			if(scale==0) {
				geometry_msgs::Point delta;
				delta.x = marker_.points[0].x-marker_.points[1].x;
				delta.y = marker_.points[0].y-marker_.points[1].y;
				delta.z = marker_.points[0].z-marker_.points[1].z;
				const float l = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
				
				scale = 0.1f*l;
			}
			
			marker_.scale.y = marker_.scale.z = marker_.scale.x = scale;
		}
		
		template<class Affine, class Box>
		void box(const Affine &pose, const Box &bb, float scale=0) {
			marker_.type = visualization_msgs::Marker::LINE_STRIP;
			
			marker_.points.clear();
			
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::BottomLeftCeil)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::BottomLeftFloor)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::BottomRightFloor)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::BottomRightCeil)));
			
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::TopRightCeil)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::TopRightFloor)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::TopLeftFloor)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::TopLeftCeil)));
			
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::BottomLeftCeil)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::BottomRightCeil)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::BottomRightFloor)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::TopRightFloor)));
			
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::TopRightCeil)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::TopLeftCeil)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::TopLeftFloor)));
			marker_.points.push_back(_2geometry(pose*bb.corner(Box::BottomLeftFloor)));
			
			if(scale==0) {
				geometry_msgs::Point delta;
				delta.x = marker_.points[0].x-marker_.points[1].x;
				delta.y = marker_.points[0].y-marker_.points[1].y;
				delta.z = marker_.points[0].z-marker_.points[1].z;
				const float l = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
				
				scale = 0.1f*l;
			}
			
			marker_.scale.y = marker_.scale.z = marker_.scale.x = scale;
		}
		
		void text(const std::string &txt, const float scale=0.1) {
			marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker_.text = txt;
			marker_.scale.z = scale;
		}
		
		template<class Vector>
		void cube(const Vector &center, const float scale=0.1) {
			primitive(center, scale, visualization_msgs::Marker::CUBE);
		}
		template<class Vector>
		void sphere(const Vector &center, const float scale=0.1) {
			primitive(center, scale, visualization_msgs::Marker::SPHERE);
		}
		template<class Vector>
		void circle(const float radius_outer, const Vector &center, const float radius_inner=0.f, const int resolution=64) {
			primitive(center, 1., visualization_msgs::Marker::TRIANGLE_LIST);

			marker_.points.resize (6*resolution);
		
			for(int i=0; i<6*resolution;) {
			  const float alpha2 = (i/6)*M_PI*2/resolution;
			  const float alpha1 = (((i/6)+1)%resolution)*M_PI*2/resolution;
			  
			  marker_.points[i+3].x = marker_.points[i].x = radius_inner*std::cos(alpha1);
			  marker_.points[i+3].y = marker_.points[i].y = radius_inner*std::sin(alpha1);
			  marker_.points[i+3].z = marker_.points[i].z = 0;
			  ++i;
			  
			  marker_.points[i+4].x = marker_.points[i].x = radius_outer*std::cos(alpha2);
			  marker_.points[i+4].y = marker_.points[i].y = radius_outer*std::sin(alpha2);
			  marker_.points[i+4].z = marker_.points[i].z = 0;
			  ++i;
			  
			  marker_.points[i].x = radius_outer*std::cos(alpha1);
			  marker_.points[i].y = radius_outer*std::sin(alpha1);
			  marker_.points[i].z = 0;
			  i+=2;
			  
			  marker_.points[i].x = radius_inner*std::cos(alpha2);
			  marker_.points[i].y = radius_inner*std::sin(alpha2);
			  marker_.points[i].z = 0;
			  i+=2;
			}
	    
		}
		
		inline RvizMarker &color(const double r, const double g,  const double b, const double a=1.) {
			marker_.color.a = a;
			marker_.color.r = r;
			marker_.color.g = g;
			marker_.color.b = b;
			return *this;
		}
		inline RvizMarker &color(const std_msgs::ColorRGBA &col) {
			marker_.color = col;
			return *this;
		}
		/// Warning: deletion of markers with different namespace than parent is not handled correctly!
		inline RvizMarker &ns(const std::string &sns) {
			marker_.ns = sns;
			return *this;
		}
		
		template<class Vector>
		void move(const Vector &v) {
			marker_.pose.position.x += v(0);
			marker_.pose.position.y += v(1);
			marker_.pose.position.z += v(2);
		}
		void move(const double x, const double y,  const double z) {
			marker_.pose.position.x += x;
			marker_.pose.position.y += y;
			marker_.pose.position.z += z;
		}
	};
}
