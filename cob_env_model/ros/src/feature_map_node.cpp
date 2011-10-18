/****************************************************************
 *
 * Copyright (c) 2011
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
 * Date of creation: 08/2011
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
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transform.h>
#include "reconfigureable_node.h"


#include <cob_env_model/feature_map_nodeConfig.h>

#include "pcl/surface/convex_hull.h"
#include "pcl/filters/project_inliers.h"


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_env_model_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

// external includes
#include <boost/timer.hpp>

// internal includes
#include "cob_env_model/map/feature_map.h"


//####################
//#### nodelet class ####
class FeatureMapNode : public pcl_ros::PCLNodelet, protected Reconfigurable_Node<cob_env_model::feature_map_nodeConfig>
{
public:

  // Constructor
  FeatureMapNode()
  : Reconfigurable_Node<cob_env_model::feature_map_nodeConfig>("FeatureMapNode")
  {
    ctr_ = 0;
    //convex_hull_sub_ = n_.subscribe("table_hull", 1, &FeatureMap::subCallback, this);
    polygon_sub_ = n_.subscribe("polygon_array", 10, &FeatureMapNode::polygonCallback, this);
    map_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("feature_map",1);
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("feature_marker",100);
    n_.param("feature_map/file_path" ,file_path_ ,std::string("/home/goa/tmp/"));
    n_.param("feature_map/save_to_file" ,save_to_file_ ,false);
    feature_map_.setFilePath(file_path_);
    feature_map_.setSaveToFile(save_to_file_);

    setReconfigureCallback(boost::bind(&callback, this, _1, _2));
  }

  // Destructor
  ~FeatureMapNode()
  {
    /// void
  }

  /**
   * @brief callback for dynamic reconfigure
   *
   * everytime the dynamic reconfiguration changes this function will be called
   *
   * @param inst instance of AggregatePointMap which parameters should be changed
   * @param config data of configuration
   * @param level bit descriptor which notifies which parameter changed
   *
   * @return nothing
   */
  static void callback(FeatureMapNode *fmn, cob_env_model::feature_map_nodeConfig &config, uint32_t level)
  {
    //TODO: not multithreading safe

    if(!fmn)
      return;

    fmn->feature_map_.setSaveToFile( config.save_to_file );
    fmn->feature_map_.setFilePath( config.file_path );
  }


  /**
   * @brief callback for adding polygons to feature map and publishing them
   *
   * callback for adding polygons to feature map and publishing them
   *
   * @param p ros message containing feature map
   *
   * @return nothing
   */
  void
  polygonCallback(const cob_env_model_msgs::PolygonArray::ConstPtr p)
  {
    FeatureMap::MapEntryPtr map_entry_ptr = FeatureMap::MapEntryPtr(new FeatureMap::MapEntry());
    convertFromROSMsg(*p, *map_entry_ptr);
    //dumpPolygonToFile(*map_entry_ptr);
    feature_map_.addMapEntry(map_entry_ptr);
    publishMapMarker();
    ctr_++;
    //ROS_INFO("%d polygons received so far", ctr_);
  }

  /**
   * @brief reading a ros message to convert it to a feature map
   *
   * reading a ros message to convert it to a feature map
   *
   * @param p ros message containing polygons
   * @param map_entry output to feature map
   *
   * @return nothing
   */
  void
  convertFromROSMsg(const cob_env_model_msgs::PolygonArray& p, FeatureMap::MapEntry& map_entry)
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

  /**
   * @brief writing to a ros message to convert a feature map
   *
   * writing to a ros message to convert a feature map
   *
   * @param p ros message containing polygons
   * @param map_entry input as feature map
   *
   * @return nothing
   */
  void
  convertToROSMsg(const FeatureMap::MapEntry& map_entry, cob_env_model_msgs::PolygonArray& p)
  {
    p.d.data = map_entry.d;
    p.normal.x = map_entry.normal(0);
    p.normal.y = map_entry.normal(1);
    p.normal.z = map_entry.normal(2);
    p.polygons.resize(map_entry.polygon_world.size());
    for(unsigned int i=0; i<map_entry.polygon_world.size(); i++)
    {
      p.polygons[i].points.resize(map_entry.polygon_world[i].size());
      for(unsigned int j=0; j<map_entry.polygon_world[i].size(); j++)
      {
        p.polygons[i].points[j].x = map_entry.polygon_world[i][j](0);
        p.polygons[i].points[j].y = map_entry.polygon_world[i][j](1);
        p.polygons[i].points[j].z = map_entry.polygon_world[i][j](2);
      }
    }
  }

  /**
   * @brief output featuremap to dump file
   *
   * output featuremap to dump file, path is hard coded
   *
   * @param m feature map
   *
   * @return nothing
   */
  void dumpPolygonToFile(FeatureMap::MapEntry& m)
  {
    static int ctr=0;
    std::stringstream ss;
    ss << "/home/goa/pcl_daten/kitchen_kinect/polygons/polygon_" << ctr << ".txt";
    std::ofstream myfile;
    myfile.open (ss.str().c_str());
    myfile << m.id << "\n";
    myfile << m.normal(0) << "\n" << m.normal(1) << "\n" << m.normal(2) << "\n";
    myfile << m.polygon_world[0].size() << "\n";
    for(unsigned int i=0; i<m.polygon_world.size(); i++)
    {
      for(unsigned int j=0; j<m.polygon_world[i].size(); j++)
      {
        myfile << m.polygon_world[i][j](0) << "\n";
        myfile << m.polygon_world[i][j](1) << "\n";
        myfile << m.polygon_world[i][j](2) << "\n";
      }
    }

    myfile.close();

    ctr++;
  }


  void publishMap()
  {
    /*for(int i=0; i<map_.size(); i++)
    {
      map_pub_.publish(map_[i].polygon_world);
    }*/
  }


  /**
   * @brief publishes the polygon of every feature
   *
   * publishes the polygon of every feature
   *
   * @return nothing
   */
  void publishMapPolygons()
  {
    geometry_msgs::PolygonStamped p;
    p.header.frame_id = "/map";
    boost::shared_ptr<std::vector<FeatureMap::MapEntryPtr> > map = feature_map_.getMap();
    for(unsigned int i=0; i<map->size(); i++)
    {
      FeatureMap::MapEntry& pm = *(map->at(i));
      for(unsigned int j=0; j<pm.polygon_world.size(); j++)
      {
        p.polygon.points.resize(pm.polygon_world[j].size());
        for(unsigned int k=0; k<pm.polygon_world[j].size(); k++)
        {
          p.polygon.points[k].x = pm.polygon_world[j][k](0);
          p.polygon.points[k].y = pm.polygon_world[j][k](1);
          p.polygon.points[k].z = pm.polygon_world[j][k](2);
        }
        map_pub_.publish(p);
      }
    }
  }

  /**
   * @brief publishes the contour of the polygons
   *
   * publishes the contour of the polygons
   *
   * @return nothing
   */
  void publishMapMarker()
  {
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration();
    marker.header.frame_id = "/map";
    //marker.header.stamp = stamp;



    //create the marker in the table reference frame
    //the caller is responsible for setting the pose of the marker to match

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 1;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1.0;

    geometry_msgs::Point pt;
    boost::shared_ptr<std::vector<FeatureMap::MapEntryPtr> > map = feature_map_.getMap();
    int ctr=0;
    for(unsigned int i=0; i<map->size(); i++)
    {
      FeatureMap::MapEntry& pm = *(map->at(i));
      //if(pm.merged/*pm.normal(2)<0.1*/)
      {
        //marker.id = pm.id;
        //if(i==0)
        {
          marker.color.r = 0;
          marker.color.g = 0;
          marker.color.b = 1;
        }
        /*else if(i==1)
        {
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 0;
        }
        else if(i==2)
        {
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 1;
        }
        else if(i==3)
        {
          marker.color.r = 1;
          marker.color.g = 1;
          marker.color.b = 0;
        }
        else if(i==4)
        {
          marker.color.r = 1;
          marker.color.g = 0;
          marker.color.b = 1;
        }
        else
        {
          marker.color.r = 0;
          marker.color.g = 0;
          marker.color.b = 1;
        }*/
        for(unsigned int j=0; j<pm.polygon_world.size(); j++)
        {
          //TODO: this is a workaround as the marker can't display more than one contour
          marker.id = ctr;
          ctr++;
          for(unsigned int k=0; k<pm.polygon_world[j].size(); k++)
          {
            marker.points.resize(pm.polygon_world[j].size()+1);
            /*pt.x = pm.polygon_world[j][k](0);
                  pt.y = pm.polygon_world[j][k](1);
                  pt.z = pm.polygon_world[j][k](2);*/
            marker.points[k].x = pm.polygon_world[j][k](0);
            marker.points[k].y = pm.polygon_world[j][k](1);
            marker.points[k].z = pm.polygon_world[j][k](2);
            //marker.points.push_back(pt);
          }
          marker.points[pm.polygon_world[j].size()].x = pm.polygon_world[j][0](0);
          marker.points[pm.polygon_world[j].size()].y = pm.polygon_world[j][0](1);
          marker.points[pm.polygon_world[j].size()].z = pm.polygon_world[j][0](2);
          marker_pub_.publish(marker);
        }
      }
    }
  }

  //    void subCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& hull)
  //	{
  //    	boost::timer t;
  //		static int ctr = 0;
  //		bool polygon_intersecting = true;
  //
  //    	if(first_)
  //    	{
  //    		first_ = false;
  //    		map_.push_back(*(hull.get()));
  //    	}
  //    	else
  //    	{
  //			//Test if hull intersects with one already in map, if yes => merge, if no => add
  //			for(unsigned int i=0; i < map_.size(); i++)
  //			{
  //				pcl::PointCloud<pcl::PointXYZ>::Ptr map_feature = map_[i].makeShared();
  //				for(unsigned int j=0; j<map_feature->points.size(); j++)
  //				{
  //					double max_x1 = 0, max_y1 = 0;
  //					double min_x1 = 0, min_y1 = 0;
  //					double max_x2 = 0, max_y2 = 0;
  //					double min_x2 = 0, min_y2 = 0;
  //					double lambda1_min = 1000, lambda1_max = -1000;
  //					double lambda2_min = 1000, lambda2_max = -1000;
  //					pcl::PointXYZ g;
  //					pcl::PointXYZ a;
  //					a.x = map_feature->points[j].x;
  //					a.y = map_feature->points[j].y;
  //					if(j<map_feature->points.size()-1)
  //					{
  //						g.x = -(map_feature->points[j+1].y - map_feature->points[j].y);
  //						g.y = map_feature->points[j+1].x - map_feature->points[j].x;
  //					}
  //					else
  //					{
  //						g.x = -(map_feature->points[0].y - map_feature->points[j].y);
  //						g.y = map_feature->points[0].x - map_feature->points[j].x;
  //					}
  //					//std::cout << "a: " << a.x << ", " << a.y << std::endl;
  //					//std::cout << "g: " << g.x << ", " << g.y << std::endl;
  //					for(unsigned int l=0; l<map_feature->points.size(); l++)
  //					{
  //						pcl::PointXYZ p = map_feature->points[l];
  //						double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
  //						//double x = a.x + m*g.x;
  //						//double y = a.y + m*g.y;
  //						if(m > lambda1_max)
  //						//if(x > max_x1)
  //						{
  //							lambda1_max = m;
  //							max_x1 = a.x + m*g.x;//x;
  //							max_y1 = a.y + m*g.y;//y;
  //						}
  //						if(m < lambda1_min)
  //						//if(x < min_x1)
  //						{
  //							lambda1_min = m;
  //							min_x1 = a.x + m*g.x;//x;
  //							min_y1 = a.y + m*g.y;//y;
  //						}
  //					}
  //					for(unsigned int k=0; k<hull->points.size(); k++)
  //					{
  //						pcl::PointXYZ p = hull->points[k];
  //						double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
  //						//double x = a.x + m*g.x;
  //						//double y = a.y + m*g.y;
  //						if(m > lambda2_max)
  //						//if(x > max_x2)
  //						{
  //							lambda2_max = m;
  //							max_x2 = a.x + m*g.x;//x;
  //							max_y2 = a.y + m*g.y;//y;
  //						}
  //						if(m < lambda2_min)
  //						//if(x < min_x2)
  //						{
  //							lambda2_min = m;
  //							min_x2 = a.x + m*g.x;//x;
  //							min_y2 = a.y + m*g.y;//y;
  //						}
  //					}
  //					//std::cout << "x1,y1; max/min:" << max_x1 << ", " << max_y1 << "; " << min_x1 << ", " << min_y1 << std::endl;
  //					//std::cout << "x2,y2; max/min:" << max_x2 << ", " << max_y2 << "; " << min_x2 << ", " << min_y2 << std::endl;
  //					//Test if intersection occurs
  //
  //					if(lambda2_min > lambda1_max || lambda1_min > lambda2_max)
  //					{
  //						polygon_intersecting = false;
  //						//std::cout << polygon_intersecting << std::endl;
  //						break;
  //					}
  //				}
  //				if(polygon_intersecting)
  //				{
  //					for(unsigned int j=0; j<hull->points.size(); j++)
  //					{
  //						double max_x1 = 0, max_y1 = 0;
  //						double min_x1 = 0, min_y1 = 0;
  //						double max_x2 = 0, max_y2 = 0;
  //						double min_x2 = 0, min_y2 = 0;
  //						double lambda1_min = 1000, lambda1_max = -1000;
  //						double lambda2_min = 1000, lambda2_max = -1000;
  //						pcl::PointXYZ g;
  //						pcl::PointXYZ a;
  //						a.x = hull->points[j].x;
  //						a.y = hull->points[j].y;
  //						if(j<hull->points.size()-1)
  //						{
  //							g.x = -(hull->points[j+1].y - hull->points[j].y);
  //							g.y = hull->points[j+1].x - hull->points[j].x;
  //						}
  //						else
  //						{
  //							g.x = -(hull->points[0].y - hull->points[j].y);
  //							g.y = hull->points[0].x - hull->points[j].x;
  //						}
  //						//std::cout << "a: " << a.x << ", " << a.y << std::endl;
  //						//std::cout << "g: " << g.x << ", " << g.y << std::endl;
  //						for(unsigned int l=0; l<hull->points.size(); l++)
  //						{
  //							pcl::PointXYZ p = hull->points[l];
  //							double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
  //							//double x = a.x + m*g.x;
  //							//double y = a.y + m*g.y;
  //							if(m > lambda1_max)
  //							//if(x > max_x1)
  //							{
  //								lambda1_max = m;
  //								max_x1 = a.x + m*g.x;//x;
  //								max_y1 = a.y + m*g.y;//y;
  //							}
  //							if(m < lambda1_min)
  //							//if(x < min_x1)
  //							{
  //								lambda1_min = m;
  //								min_x1 = a.x + m*g.x;//x;
  //								min_y1 = a.y + m*g.y;//y;
  //							}
  //						}
  //						for(unsigned int k=0; k<map_feature->points.size(); k++)
  //						{
  //							pcl::PointXYZ p = map_feature->points[k];
  //							double m = (p.x*g.x+p.y*g.y-a.x*g.x-a.y*g.y)/(g.x*g.x+g.y*g.y);
  //							//double x = a.x + m*g.x;
  //							//double y = a.y + m*g.y;
  //							if(m > lambda2_max)
  //							//if(x > max_x2)
  //							{
  //								lambda2_max = m;
  //								max_x2 = a.x + m*g.x;//x;
  //								max_y2 = a.y + m*g.y;//y;
  //							}
  //							if(m < lambda2_min)
  //							//if(x < min_x2)
  //							{
  //								lambda2_min = m;
  //								min_x2 = a.x + m*g.x;//x;
  //								min_y2 = a.y + m*g.y;//y;
  //							}
  //						}
  //						//std::cout << "x1,y1; max/min:" << max_x1 << ", " << max_y1 << "; " << min_x1 << ", " << min_y1 << std::endl;
  //						//std::cout << "x2,y2; max/min:" << max_x2 << ", " << max_y2 << "; " << min_x2 << ", " << min_y2 << std::endl;
  //						//Test if intersection occurs
  //
  //						if(lambda2_min > lambda1_max || lambda1_min > lambda2_max)
  //						{
  //							polygon_intersecting = false;
  //							//std::cout << polygon_intersecting << std::endl;
  //							break;
  //						}
  //					}
  //				}
  //				if(polygon_intersecting)
  //				{
  //					std::cout << "Polygon intersecting: " << polygon_intersecting << std::endl;
  //					map_[i] += *(hull.get());
  //
  //				   // Create a set of planar coefficients with X=Y=0,Z=1
  //				   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  //				   coefficients->values.resize (4);
  //				   coefficients->values[0] = coefficients->values[1] = 0;
  //				   coefficients->values[2] = 1.0;
  //				   coefficients->values[3] = -hull->points[0].z;
  //
  //					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
  //					pcl::ProjectInliers<pcl::PointXYZ> proj;
  //					proj.setModelType (pcl::SACMODEL_PLANE);
  //					proj.setInputCloud (map_[i].makeShared());
  //					proj.setModelCoefficients (coefficients);
  //					proj.filter (*cloud_projected);
  //					// Create a Convex Hull representation of the projected inliers
  //					pcl::ConvexHull<pcl::PointXYZ> chull;
  //					pcl::PointCloud<pcl::PointXYZ> hull_old;
  //					//pcl::io::loadPCDFile("/home/goa/pcl_daten/feature_map/map_1_f0.pcd", hull_old);
  //					pcl::PointCloud<pcl::PointXYZ> hull_new;
  //					chull.setInputCloud (cloud_projected);
  //					chull.reconstruct (map_[i]);
  //					std::stringstream ss1;
  //					//ss1 << "/home/goa/pcl_daten/feature_map/hull_new_" << ctr << ".pcd";
  //					//pcl::io::savePCDFileASCII (ss1.str(), hull_new/*map_[i]*/);
  //
  //					break;
  //				}
  //			}
  //			if(!polygon_intersecting)
  //			{
  //				map_.push_back(*(hull.get()));
  //				std::cout << "appending" << std::endl;
  //			}
  //    	}
  //
  //
  //		ctr++;
  //
  //		std::cout << "time: " << t.elapsed() << std::endl;
  //		publishMarker();
  //
  //		return;
  //	}

  //    void publishMarker()
  //    {
  //    	static int ctr;
  //    	visualization_msgs::MarkerArray marker_array;
  //    	marker_array.markers.resize(map_.size());
  //    	//std::cout << "map size:" << map_.size() << std::endl;
  //    	for(unsigned int j=0; j<map_.size();j++)
  //    	{
  //			visualization_msgs::Marker marker;
  //			marker.action = visualization_msgs::Marker::ADD;
  //			marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  //			marker.lifetime = ros::Duration(0);
  //			marker.header.frame_id = map_[j].header.frame_id;
  //			marker.header.stamp = ros::Time::now();
  //			marker.id = ctr++;
  //
  //			marker.scale.x = 1;
  //			marker.scale.y = 1;
  //			marker.scale.z = 1;
  //
  //			geometry_msgs::Point pt1, pt2, pt3;
  //			pt1.x = map_[j].points[0].x;
  //			pt1.y = map_[j].points[0].y;
  //			pt1.z = map_[j].points[0].z;
  //			//std::cout << j << std::endl;
  //
  //			for(unsigned int i = 1; i < map_[j].points.size()-1; i++)
  //			{
  //				pt2.x = map_[j].points[i].x;
  //				pt2.y = map_[j].points[i].y;
  //				pt2.z = map_[j].points[i].z;
  //
  //				pt3.x = map_[j].points[i+1].x;
  //				pt3.y = map_[j].points[i+1].y;
  //				pt3.z = map_[j].points[i+1].z;
  //
  //				marker.points.push_back(pt1);
  //				marker.points.push_back(pt2);
  //				marker.points.push_back(pt3);
  //			}
  //			//std::cout << "marker size: " << marker.points.size() << std::endl;
  //
  //			marker.color.r = 0.0;
  //			marker.color.g = 1.0;
  //			marker.color.b = 0.0;
  //			marker.color.a = 1.0;
  //			marker_array.markers[j]=marker;
  //			marker_pub_.publish(marker);
  //    	}
  //		//marker_pub_.publish(marker_array);
  //    }

  ros::NodeHandle n_;


protected:
  ros::Subscriber polygon_sub_;
  ros::Publisher map_pub_;
  ros::Publisher marker_pub_;

  FeatureMap feature_map_;      /// map containing features (polygons)

  unsigned int ctr_;            /// counter how many polygons are received
  std::string file_path_;
  bool save_to_file_;
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "feature_map_node");

  FeatureMapNode fmn;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep();
  }
}

//PLUGINLIB_DECLARE_CLASS(cob_env_model, FeatureMap, FeatureMap, nodelet::Nodelet)

