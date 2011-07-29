/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


//##################
//#### includes ####

// standard includes
//--
#include <sstream>

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "pcl/surface/convex_hull.h"
#include "pcl/filters/project_inliers.h"

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// external includes
#include <boost/timer.hpp>


//####################
//#### nodelet class ####
class FeatureMap : public pcl_ros::PCLNodelet
{
public:
    // Constructor
	FeatureMap()
	:first_(true)
	{
	}

    // Destructor
    ~FeatureMap()
    {
    	/// void
    }


    void onInit()
    {
    	PCLNodelet::onInit();
    	n_ = getNodeHandle();

    	convex_hull_sub_ = n_.subscribe("table_hull", 1, &FeatureMap::subCallback, this);
		map_pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("feature_map",1);
		marker_pub_ = n_.advertise<visualization_msgs::Marker>("feature_marker",100);

    	/*pcl::PointCloud<pcl::PointXYZ> map_feature;
    	map_feature.points.resize(3);
    	map_feature.points[0].x = 0;
    	map_feature.points[0].y = 0;
    	map_feature.points[0].z = 0;
    	map_feature.points[1].x = 1;
    	map_feature.points[1].y = 0.5;
    	map_feature.points[1].z = 0;
    	map_feature.points[2].x = 0;
    	map_feature.points[2].y = 1;
    	map_feature.points[2].z = 0;
    	map_feature.header.frame_id="/head_tof_link";*/
    	//pcl::io::loadPCDFile("/home/goa/pcl_daten/table_detection/hull_0.pcd", map_feature);
    	//map_.push_back(map_feature);
    }


    void subCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& hull)
	{
    	boost::timer t;
		static int ctr = 0;
		bool polygon_intersecting = true;
		/*hull->points.resize(4);
		hull->points[0].x = 0.5;
		hull->points[0].y = 0;
		hull->points[0].z = 0;
		hull->points[1].x = 1.5;
		hull->points[1].y = 0.5;
		hull->points[1].z = 0;
		hull->points[2].x = 1;
		hull->points[2].y = 1.5;
		hull->points[2].z = 0;
		hull->points[3].x = 0.5;
		hull->points[3].y = 1.5;
		hull->points[3].z = 0;*/
		/*std::stringstream ss2;
		ss2 << "/home/goa/pcl_daten/table_detection/hull_" << ctr << ".pcd";
		pcl::io::loadPCDFile(ss2.str(), *(hull.get()));*/
    	if(first_)
    	{
    		first_ = false;
    		map_.push_back(*(hull.get()));
    	}
    	else
    	{
			//Test if hull intersects with one already in map, if yes => merge, if no => add
			for(unsigned int i=0; i < map_.size(); i++)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr map_feature = map_[i].makeShared();
				for(unsigned int j=0; j<map_feature->points.size(); j++)
				{
					double max_x1 = 0, max_y1 = 0;
					double min_x1 = 0, min_y1 = 0;
					double max_x2 = 0, max_y2 = 0;
					double min_x2 = 0, min_y2 = 0;
					double lambda1_min = 1000, lambda1_max = -1000;
					double lambda2_min = 1000, lambda2_max = -1000;
					pcl::PointXYZ g;
					pcl::PointXYZ a;
					a.x = map_feature->points[j].x;
					a.y = map_feature->points[j].y;
					if(j<map_feature->points.size()-1)
					{
						g.x = -(map_feature->points[j+1].y - map_feature->points[j].y);
						g.y = map_feature->points[j+1].x - map_feature->points[j].x;
					}
					else
					{
						g.x = -(map_feature->points[0].y - map_feature->points[j].y);
						g.y = map_feature->points[0].x - map_feature->points[j].x;
					}
					//std::cout << "a: " << a.x << ", " << a.y << std::endl;
					//std::cout << "g: " << g.x << ", " << g.y << std::endl;
					for(unsigned int l=0; l<map_feature->points.size(); l++)
					{
						pcl::PointXYZ p = map_feature->points[l];
						double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
						//double x = a.x + m*g.x;
						//double y = a.y + m*g.y;
						if(m > lambda1_max)
						//if(x > max_x1)
						{
							lambda1_max = m;
							max_x1 = a.x + m*g.x;//x;
							max_y1 = a.y + m*g.y;//y;
						}
						if(m < lambda1_min)
						//if(x < min_x1)
						{
							lambda1_min = m;
							min_x1 = a.x + m*g.x;//x;
							min_y1 = a.y + m*g.y;//y;
						}
					}
					for(unsigned int k=0; k<hull->points.size(); k++)
					{
						pcl::PointXYZ p = hull->points[k];
						double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
						//double x = a.x + m*g.x;
						//double y = a.y + m*g.y;
						if(m > lambda2_max)
						//if(x > max_x2)
						{
							lambda2_max = m;
							max_x2 = a.x + m*g.x;//x;
							max_y2 = a.y + m*g.y;//y;
						}
						if(m < lambda2_min)
						//if(x < min_x2)
						{
							lambda2_min = m;
							min_x2 = a.x + m*g.x;//x;
							min_y2 = a.y + m*g.y;//y;
						}
					}
					//std::cout << "x1,y1; max/min:" << max_x1 << ", " << max_y1 << "; " << min_x1 << ", " << min_y1 << std::endl;
					//std::cout << "x2,y2; max/min:" << max_x2 << ", " << max_y2 << "; " << min_x2 << ", " << min_y2 << std::endl;
					//Test if intersection occurs
					/*if((max_x2-min_x1)/(max_x1-max_x2) <0 && (min_x2-min_x1)/(max_x1-min_x2) <0 &&
							(max_y2-min_y1)/(max_y1-max_y2) <0 && (min_y2-min_y1)/(max_y1-min_y2) <0 &&
							(max_x1-min_x2)/(max_x2-max_x1) <0 && (min_x1-min_x2)/(max_x2-min_x1) <0 &&
							(max_y1-min_y2)/(max_y2-max_y1) <0 && (min_y1-min_y2)/(max_y2-min_y1) <0) //min 1 inner point*/
					if(lambda2_min > lambda1_max || lambda1_min > lambda2_max)
					{
						polygon_intersecting = false;
						//std::cout << polygon_intersecting << std::endl;
						break;
					}
				}
				if(polygon_intersecting)
				{
					for(unsigned int j=0; j<hull->points.size(); j++)
					{
						double max_x1 = 0, max_y1 = 0;
						double min_x1 = 0, min_y1 = 0;
						double max_x2 = 0, max_y2 = 0;
						double min_x2 = 0, min_y2 = 0;
						double lambda1_min = 1000, lambda1_max = -1000;
						double lambda2_min = 1000, lambda2_max = -1000;
						pcl::PointXYZ g;
						pcl::PointXYZ a;
						a.x = hull->points[j].x;
						a.y = hull->points[j].y;
						if(j<hull->points.size()-1)
						{
							g.x = -(hull->points[j+1].y - hull->points[j].y);
							g.y = hull->points[j+1].x - hull->points[j].x;
						}
						else
						{
							g.x = -(hull->points[0].y - hull->points[j].y);
							g.y = hull->points[0].x - hull->points[j].x;
						}
						//std::cout << "a: " << a.x << ", " << a.y << std::endl;
						//std::cout << "g: " << g.x << ", " << g.y << std::endl;
						for(unsigned int l=0; l<hull->points.size(); l++)
						{
							pcl::PointXYZ p = hull->points[l];
							double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
							//double x = a.x + m*g.x;
							//double y = a.y + m*g.y;
							if(m > lambda1_max)
							//if(x > max_x1)
							{
								lambda1_max = m;
								max_x1 = a.x + m*g.x;//x;
								max_y1 = a.y + m*g.y;//y;
							}
							if(m < lambda1_min)
							//if(x < min_x1)
							{
								lambda1_min = m;
								min_x1 = a.x + m*g.x;//x;
								min_y1 = a.y + m*g.y;//y;
							}
						}
						for(unsigned int k=0; k<map_feature->points.size(); k++)
						{
							pcl::PointXYZ p = map_feature->points[k];
							double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
							//double x = a.x + m*g.x;
							//double y = a.y + m*g.y;
							if(m > lambda2_max)
							//if(x > max_x2)
							{
								lambda2_max = m;
								max_x2 = a.x + m*g.x;//x;
								max_y2 = a.y + m*g.y;//y;
							}
							if(m < lambda2_min)
							//if(x < min_x2)
							{
								lambda2_min = m;
								min_x2 = a.x + m*g.x;//x;
								min_y2 = a.y + m*g.y;//y;
							}
						}
						//std::cout << "x1,y1; max/min:" << max_x1 << ", " << max_y1 << "; " << min_x1 << ", " << min_y1 << std::endl;
						//std::cout << "x2,y2; max/min:" << max_x2 << ", " << max_y2 << "; " << min_x2 << ", " << min_y2 << std::endl;
						//Test if intersection occurs
						/*if((max_x2-min_x1)/(max_x1-max_x2) <0 && (min_x2-min_x1)/(max_x1-min_x2) <0 &&
								(max_y2-min_y1)/(max_y1-max_y2) <0 && (min_y2-min_y1)/(max_y1-min_y2) <0 &&
								(max_x1-min_x2)/(max_x2-max_x1) <0 && (min_x1-min_x2)/(max_x2-min_x1) <0 &&
								(max_y1-min_y2)/(max_y2-max_y1) <0 && (min_y1-min_y2)/(max_y2-min_y1) <0) //min 1 inner point*/
						if(lambda2_min > lambda1_max || lambda1_min > lambda2_max)
						{
							polygon_intersecting = false;
							//std::cout << polygon_intersecting << std::endl;
							break;
						}
					}
				}
				if(polygon_intersecting)
				{
					std::cout << "Polygon intersecting: " << polygon_intersecting << std::endl;
					map_[i] += *(hull.get());

				   // Create a set of planar coefficients with X=Y=0,Z=1
				   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
				   coefficients->values.resize (4);
				   coefficients->values[0] = coefficients->values[1] = 0;
				   coefficients->values[2] = 1.0;
				   coefficients->values[3] = -hull->points[0].z;

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
					pcl::ProjectInliers<pcl::PointXYZ> proj;
					proj.setModelType (pcl::SACMODEL_PLANE);
					proj.setInputCloud (map_[i].makeShared());
					proj.setModelCoefficients (coefficients);
					proj.filter (*cloud_projected);
					// Create a Convex Hull representation of the projected inliers
					pcl::ConvexHull<pcl::PointXYZ> chull;
					pcl::PointCloud<pcl::PointXYZ> hull_old;
					//pcl::io::loadPCDFile("/home/goa/pcl_daten/feature_map/map_1_f0.pcd", hull_old);
					pcl::PointCloud<pcl::PointXYZ> hull_new;
					chull.setInputCloud (cloud_projected);
					chull.reconstruct (map_[i]);
					std::stringstream ss1;
					//ss1 << "/home/goa/pcl_daten/feature_map/hull_new_" << ctr << ".pcd";
					//pcl::io::savePCDFileASCII (ss1.str(), hull_new/*map_[i]*/);
					/*for(unsigned int v=0; v<map_[i].points.size(); v++)
					{
						std::cout << map_[i].points[v] << ", ";
					}
					std::cout <<  std::endl;*/
					break;
				}
			}
			if(!polygon_intersecting)
			{
				map_.push_back(*(hull.get()));
				std::cout << "appending" << std::endl;
			}
    	}

		/*for(unsigned int i=0; i<map_.size(); i++)
		{
			std::stringstream ss;
			ss << "/home/goa/pcl_daten/table/feature_map/map_" << ctr << "_f" << i << ".pcd";
			pcl::io::savePCDFileASCII (ss.str(), map_[i]);
    	}*/
		ctr++;

		std::cout << "time: " << t.elapsed() << std::endl;
		publishMarker();

		return;
	}

    void publishMarker()
    {
    	static int ctr;
    	visualization_msgs::MarkerArray marker_array;
    	marker_array.markers.resize(map_.size());
    	//std::cout << "map size:" << map_.size() << std::endl;
    	for(unsigned int j=0; j<map_.size();j++)
    	{
			visualization_msgs::Marker marker;
			marker.action = visualization_msgs::Marker::ADD;
			marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
			marker.lifetime = ros::Duration(0);
			marker.header.frame_id = map_[j].header.frame_id;
			marker.header.stamp = ros::Time::now();
			marker.id = ctr++;

			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 1;

			geometry_msgs::Point pt1, pt2, pt3;
			pt1.x = map_[j].points[0].x;
			pt1.y = map_[j].points[0].y;
			pt1.z = map_[j].points[0].z;
			//std::cout << j << std::endl;

			for(unsigned int i = 1; i < map_[j].points.size()-1; i++)
			{
				pt2.x = map_[j].points[i].x;
				pt2.y = map_[j].points[i].y;
				pt2.z = map_[j].points[i].z;

				pt3.x = map_[j].points[i+1].x;
				pt3.y = map_[j].points[i+1].y;
				pt3.z = map_[j].points[i+1].z;

				marker.points.push_back(pt1);
				marker.points.push_back(pt2);
				marker.points.push_back(pt3);
			}
			//std::cout << "marker size: " << marker.points.size() << std::endl;

			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;
			marker_array.markers[j]=marker;
			marker_pub_.publish(marker);
    	}
		//marker_pub_.publish(marker_array);
    }

    ros::NodeHandle n_;


protected:
    ros::Subscriber convex_hull_sub_;
    ros::Publisher map_pub_;
    ros::Publisher marker_pub_;

    std::vector<pcl::PointCloud<pcl::PointXYZ> > map_;

    bool first_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, FeatureMap, FeatureMap, nodelet::Nodelet)

