#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <list>
#include <string>

/* 
 * EXAMPLE USAGE:
 * 
 */

namespace cob_3d_visualization {
	
	class RvizMarkerManager {
		std::string ns_, frame_id_;
		int id_;
		std::list<int> active_ids_;
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
		int newId() {active_ids_.push_back(id_); return id_++;}
		
		RvizMarkerManager &createTopic(const std::string &topic_name) {
			ros::NodeHandle nh;
			vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>( topic_name, 0 );
			return *this;
		}
		void prepare(const visualization_msgs::Marker &marker) {markers_.markers.push_back(marker);}
		void publish() {
			vis_pub_.publish(markers_);
			markers_.markers.clear();
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
				active_ids_.push_back(mi);
				
			ros::Rate poll_rate(20);
			int num = 0;
			while(vis_pub_.getNumSubscribers() == 0 && (num++)<100)
				poll_rate.sleep();
				
			if(vis_pub_.getNumSubscribers() == 0)
				ROS_WARN("could not clear markers, as no subscribers are present at the moment");
		
			clear();
		}
		
		void clear() {
			if(active_ids_.size()>0) {
				ros::NodeHandle nh;
				nh.setParam("/simple_marker/"+ns()+"/min", active_ids_.back());
			}
			
			while(active_ids_.size()>0) {
				visualization_msgs::Marker marker;
				marker.header.frame_id = frame_id();
				marker.header.stamp = ros::Time();
				marker.ns = ns();
				marker.id = active_ids_.back();
				marker.action = visualization_msgs::Marker::DELETE;
				
				active_ids_.pop_back();
				prepare(marker);
			}
			
			publish();
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
		
		void mesh(const pcl::PolygonMesh &mesh) {
			pcl::PointCloud<pcl::PointXYZ> points;
			pcl::fromROSMsg(mesh.cloud, points);
			for(size_t i=0; i<mesh.polygons.size(); i++)
				for(int j=0; j<(int)mesh.polygons[i].vertices.size()-2; j++) {
					addTriangle(
						points[mesh.polygons[i].vertices[j  ]].getVector3fMap(),
						points[mesh.polygons[i].vertices[j+1]].getVector3fMap(),
						points[mesh.polygons[i].vertices[j+2]].getVector3fMap()
					);
			}
		}
		
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

			marker.points.resize (6*resolution);
		
			for(int i=0; i<6*resolution;) {
			  const float alpha1 = i*M_PI*2/resolution;
			  const float alpha2 = ((i+1)%resolution)*M_PI*2/resolution;
			  
			  marker.points[i+3].x = marker.points[i].x = radius_inner*std::cos(alpha1);
			  marker.points[i+3].y = marker.points[i].y = radius_inner*std::cos(alpha1);
			  marker.points[i+3].z = marker.points[i].z = 0;
			  ++i;
			  
			  marker.points[i+4].x = marker.points[i].x = radius_outer*std::cos(alpha2);
			  marker.points[i+4].x = marker.points[i].y = radius_outer*std::cos(alpha2);
			  marker.points[i+4].x = marker.points[i].z = 0;
			  ++i;
			  
			  marker.points[i].x = radius_outer*std::cos(alpha1);
			  marker.points[i].y = radius_outer*std::cos(alpha1);
			  marker.points[i].z = 0;
			  i+=2;
			  
			  marker.points[i].x = radius_inner*std::cos(alpha2);
			  marker.points[i].y = radius_inner*std::cos(alpha2);
			  marker.points[i].z = 0;
			  i+=2;
			}
	    
		}
		
		inline void color(const double r, const double g,  const double b, const double a=1.) {
			marker_.color.a = a;
			marker_.color.r = r;
			marker_.color.g = g;
			marker_.color.b = b;
		}
		inline void color(const std_msgs::ColorRGBA &col) {
			marker_.color = col;
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
