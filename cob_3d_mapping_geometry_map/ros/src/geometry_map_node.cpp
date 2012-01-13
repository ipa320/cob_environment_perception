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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transform.h>
#include <cob_3d_mapping_common/reconfigureable_node.h>


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
#include <geometry_msgs/PolygonStamped.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>

// internal includes
#include "cob_3d_mapping_geometry_map/geometry_map.h"
#include "cob_3d_mapping_geometry_map/map_entry.h"


//####################
//#### nodelet class ####
class GeometryMapNode : protected Reconfigurable_Node<cob_3d_mapping_geometry_map::geometry_map_nodeConfig>
{
public:

  // Constructor
	GeometryMapNode()
  : Reconfigurable_Node<cob_3d_mapping_geometry_map::geometry_map_nodeConfig>("GeometryMapNode")
  {
    ctr_ = 0;
    //convex_hull_sub_ = n_.subscribe("table_hull", 1, &FeatureMap::subCallback, this);
    polygon_sub_ = n_.subscribe("polygon_array", 10, &GeometryMapNode::polygonCallback, this);
    map_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("geometry_map",1);
    map_pub_2_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray>("geometry_map_2",1);
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("geometry_marker",100);
    clear_map_server_ = n_.advertiseService("clear_geometry_map", &GeometryMapNode::clearMap, this);
    get_map_server_ = n_.advertiseService("get_geometry_map", &GeometryMapNode::getMap, this);
    ros::param::param("~file_path" ,file_path_ ,std::string("/home/goa/tmp/"));
    ros::param::param("~save_to_file" ,save_to_file_ ,false);
    std::cout << file_path_ << std::endl;
    geometry_map_.setFilePath(file_path_);
    geometry_map_.setSaveToFile(save_to_file_);

    setReconfigureCallback(boost::bind(&callback, this, _1, _2));
  }

  // Destructor
  ~GeometryMapNode()
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
  static void callback(GeometryMapNode *gmn, cob_3d_mapping_geometry_map::geometry_map_nodeConfig &config, uint32_t level)
  {
    //TODO: not multithreading safe

    if(!gmn)
      return;

    gmn->geometry_map_.setSaveToFile( config.save_to_file );
    gmn->geometry_map_.setFilePath( config.file_path );
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
  polygonCallback(const cob_3d_mapping_msgs::PolygonArray::ConstPtr p)
  {
	MapEntryPtr map_entry_ptr = MapEntryPtr(new MapEntry());
    convertFromROSMsg(*p, *map_entry_ptr);
    //dumpPolygonToFile(*map_entry_ptr);
    geometry_map_.addMapEntry(map_entry_ptr);
    publishMapMarker();
    publishMap();
    ctr_++;
    //ROS_INFO("%d polygons received so far", ctr_);
  }

  /**
   * @brief clears map
   *
   * deletes 3d map of the environment
   *
   * @param req not needed
   * @param res not needed
   *
   * @return nothing
   */
  bool
  clearMap(cob_srvs::Trigger::Request &req,
           cob_srvs::Trigger::Response &res)
  {
    //TODO: add mutex
    ROS_INFO("Clearing geometry map...");
    geometry_map_.clearMap();
    return true;
  }

  /**
   * @brief service callback for GetGeometricMap service
   *
   * Fills the service response of the GetGeometricMap service with the current point map
   *
   * @param req request to send map
   * @param res the current geometric map
   *
   * @return nothing
   */
  bool
  getMap(cob_3d_mapping_msgs::GetGeometricMap::Request &req,
         cob_3d_mapping_msgs::GetGeometricMap::Response &res)
  {
    boost::shared_ptr<std::vector<MapEntryPtr> > map = geometry_map_.getMap();
    for(unsigned int i=0; i<map->size(); i++)
    {
      MapEntry& sm = *(map->at(i));
      cob_3d_mapping_msgs::Shape s;
      convertToROSMsg(sm,s);
      res.map.shapes.push_back(s);
    }
    return true;
  }


  /**
   * @brief reading a ros message to convert it to a feature map
   *
   * reading a ros message to convert it to a geometry map
   *
   * @param p ros message containing polygons
   * @param map_entry output to geometry map
   *
   * @return nothing
   */
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

  /**
   * @brief writing to a ros message to convert a geometry map
   *
   * writing to a ros message to convert a geometry map
   *
   * @param p ros message containing polygons
   * @param map_entry input as geometry map
   *
   * @return nothing
   */
  void
  convertToROSMsg(const MapEntry& map_entry, cob_3d_mapping_msgs::PolygonArray& p)
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
  convertToROSMsg(const MapEntry& map_entry, cob_3d_mapping_msgs::Shape& s)
  {
    s.params.resize(4);
    s.params[0] = map_entry.normal(0);
    s.params[1] = map_entry.normal(1);
    s.params[2] = map_entry.normal(2);
    s.params[3] = map_entry.d;
    s.points.resize(map_entry.polygon_world.size());
    for(unsigned int i=0; i<map_entry.polygon_world.size(); i++)
    {
      //s.points[i].points.resize(map_entry.polygon_world[i].size());
      pcl::PointCloud<pcl::PointXYZ> cloud;
      for(unsigned int j=0; j<map_entry.polygon_world[i].size(); j++)
      {
        pcl::PointXYZ p;
        p.x = map_entry.polygon_world[i][j](0);
        p.y = map_entry.polygon_world[i][j](1);
        p.z = map_entry.polygon_world[i][j](2);
        cloud.points.push_back(p);
        /*s.points[i].points[j].x = map_entry.polygon_world[i][j](0);
        s.points[i].points[j].y = map_entry.polygon_world[i][j](1);
        s.points[i].points[j].z = map_entry.polygon_world[i][j](2);*/
      }
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud, cloud_msg);
      s.points[i]= cloud_msg;
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
  void dumpPolygonToFile(MapEntry& m)
  {
    static int ctr=0;
    std::stringstream ss;
    ss << "/home/goa-hh/pcl_daten/kitchen_kinect/polygons/polygon_" << ctr << ".txt";
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
    boost::shared_ptr<std::vector<MapEntryPtr> > map = geometry_map_.getMap();
    //cob_3d_mapping_msgs::PolygonArrayArray map_msg;
    cob_3d_mapping_msgs::ShapeArray map_msg;
    for(unsigned int i=0; i<map->size(); i++)
    {
      MapEntry& sm = *(map->at(i));
      //cob_3d_mapping_msgs::PolygonArray p;
      cob_3d_mapping_msgs::Shape s;
      convertToROSMsg(sm, s);
      //map_msg.polygon_array.push_back(p);
      map_msg.shapes.push_back(s);
    }
    map_pub_2_.publish(map_msg);
  }


  /**
   * @brief publishes the polygon of every geometry
   *
   * publishes the polygon of every geometry
   *
   * @return nothing
   */
  void publishMapPolygons()
  {
    geometry_msgs::PolygonStamped p;
    p.header.frame_id = "/map";
    boost::shared_ptr<std::vector<MapEntryPtr> > map = geometry_map_.getMap();
    for(unsigned int i=0; i<map->size(); i++)
    {
    	MapEntry& pm = *(map->at(i));
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
    boost::shared_ptr<std::vector<MapEntryPtr> > map = geometry_map_.getMap();
    int ctr=0;
    for(unsigned int i=0; i<map->size(); i++)
    {
    	MapEntry& pm = *(map->at(i));
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

  ros::NodeHandle n_;


protected:
  ros::Subscriber polygon_sub_;
  ros::Publisher map_pub_;
  ros::Publisher map_pub_2_;
  ros::Publisher marker_pub_;
  ros::ServiceServer clear_map_server_;
  ros::ServiceServer get_map_server_;

  GeometryMap geometry_map_;      /// map containing geometrys (polygons)

  unsigned int ctr_;            /// counter how many polygons are received
  std::string file_path_;
  bool save_to_file_;
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "geometry_map_node");

  GeometryMapNode gmn;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep();
  }
}

//PLUGINLIB_DECLARE_CLASS(cob_env_model, FeatureMap, FeatureMap, nodelet::Nodelet)

