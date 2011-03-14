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
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "pcl/surface/convex_hull.h"


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>


//####################
//#### nodelet class ####
class FeatureMap : public pcl_ros::PCLNodelet
{
public:
    // Constructor
	FeatureMap()
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

    	pcl::PointCloud<pcl::PointXYZ> map_feature;
    	map_feature.points.resize(3);
    	map_feature.points[0].x = 0;
    	map_feature.points[0].y = 0;
    	map_feature.points[1].x = 1;
    	map_feature.points[1].y = 0.5;
    	map_feature.points[2].x = 0;
    	map_feature.points[2].y = 1;
    	map_.push_back(map_feature);
    }


    void subCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& hull)
	{
    	hull->points.resize(3);
    	hull->points[0].x = 4;
    	hull->points[0].y = 4;
    	hull->points[1].x = 5;
    	hull->points[1].y = 5;
    	hull->points[2].x = 4;
    	hull->points[2].y = 5;
		bool polygon_intersecting = true;
		//Test if hull intersects with one already in map, if yes => merge, if no => add
		for(unsigned int i=0; i < map_.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr map_feature = map_[i].makeShared();
			for(unsigned int j=0; j<map_feature->points.size(); j++)
			{
				double max_x1 = 0, max_y1 = 0;
				double min_x1 = 1000, min_y1 = 1000;
				double max_x2 = 0, max_y2 = 0;
				double min_x2 = 1000, min_y2 = 1000;
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
				std::cout << "a: " << a.x << ", " << a.y << std::endl;
				std::cout << "g: " << g.x << ", " << g.y << std::endl;
				for(unsigned int l=0; l<map_feature->points.size(); l++)
				{
					pcl::PointXYZ p = map_feature->points[l];
					double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
					double x = a.x + m*g.x;
					double y = a.y + m*g.y;
					if(x > max_x1)
					{
						max_x1 = x;
						max_y1 = y;
					}
					if(x < min_x1)
					{
						min_x1 = x;
						min_y1 = y;
					}
				}
				for(unsigned int k=0; k<hull->points.size(); k++)
				{
					pcl::PointXYZ p = hull->points[k];
					double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
					double x = a.x + m*g.x;
					double y = a.y + m*g.y;
					if(x > max_x2)
					{
						max_x2 = x;
						max_y2 = y;
					}
					if(x < min_x2)
					{
						min_x2 = x;
						min_y2 = y;
					}
				}
				std::cout << "x1,y1; max/min:" << max_x1 << ", " << max_y1 << "; " << min_x1 << ", " << min_y1 << std::endl;
				std::cout << "x2,y2; max/min:" << max_x2 << ", " << max_y2 << "; " << min_x2 << ", " << min_y2 << std::endl;
				//Test if intersection occurs
				if((max_x2-min_x1)/(max_x1-max_x2) <0 || (min_x2-min_x1)/(max_x1-min_x2) <0 ||
						(max_y2-min_y1)/(max_y1-max_y2) <0 || (min_y2-min_y1)/(max_y1-min_y2) <0) //äußerer Teilpunkt, separating line found
				{
					polygon_intersecting = false;
					std::cout << polygon_intersecting << std::endl;
					break;
				}
			}
			std::cout << polygon_intersecting << std::endl;
			//Do the same the other way round.
		}

		return;
	}

    ros::NodeHandle n_;


protected:
    ros::Subscriber convex_hull_sub_;
    ros::Publisher map_pub_;

    std::vector<pcl::PointCloud<pcl::PointXYZ> > map_;

};

PLUGINLIB_DECLARE_CLASS(cob_env_model, FeatureMap, FeatureMap, nodelet::Nodelet)

