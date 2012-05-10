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
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <dynamic_reconfigure/server.h>
#include <cob_3d_mapping_geometry_map/geometry_map_nodeConfig.h>

#include "pcl/surface/convex_hull.h"
#include "pcl/filters/project_inliers.h"


// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_3d_mapping_msgs/GetGeometricMap.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"

// internal includes
#include "cob_3d_mapping_geometry_map/geometry_map.h"

using namespace cob_3d_mapping;

//####################
//#### nodelet class ####
class GeometryMapNode //: protected Reconfigurable_Node<cob_3d_mapping_geometry_map::geometry_map_nodeConfig>
{
public:

  // Constructor
  GeometryMapNode()
  {
    config_server_.setCallback(boost::bind(&GeometryMapNode::dynReconfCallback, this, _1, _2));
    ctr_ = 0;
    shape_sub_ = n_.subscribe("shape_array", 10, &GeometryMapNode::shapeCallback, this);
    map_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray>("map_array",1);
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("geometry_marker",100);
    clear_map_server_ = n_.advertiseService("clear_map", &GeometryMapNode::clearMap, this);
    get_map_server_ = n_.advertiseService("get_map", &GeometryMapNode::getMap, this);
    ros::param::param("~file_path" ,file_path_ ,std::string("/home/goa/tmp/"));
    ros::param::param("~save_to_file" ,save_to_file_ ,false);
    std::cout << file_path_ << std::endl;
    geometry_map_.setFilePath(file_path_);
    geometry_map_.setSaveToFile(save_to_file_);
  }

  // Destructor
  ~GeometryMapNode()
  {
    /// void
  }

  void dynReconfCallback(cob_3d_mapping_geometry_map::geometry_map_nodeConfig &config, uint32_t level)
  {
    geometry_map_.setSaveToFile( config.save_to_file );
    geometry_map_.setFilePath( config.file_path );
    geometry_map_.setMergeThresholds(config.cos_angle, config.d);
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
  /*static void callback(GeometryMapNode *gmn, cob_3d_mapping_geometry_map::geometry_map_nodeConfig &config, uint32_t level)
  {
    //TODO: not multithreading safe

    if(!gmn)
      return;

    gmn->geometry_map_.setSaveToFile( config.save_to_file );
    gmn->geometry_map_.setFilePath( config.file_path );
  }*/


  void
  shapeCallback(const cob_3d_mapping_msgs::ShapeArray::ConstPtr sa)
  {
    static int ctr=0;
    static double time = 0;
    PrecisionStopWatch t;
    for(unsigned int i=0; i<sa->shapes.size(); i++)
    {
      PolygonPtr map_entry_ptr = PolygonPtr(new Polygon());
      if(!fromROSMsg(sa->shapes[i], *map_entry_ptr)) continue;
      //dumpPolygonToFile(*map_entry_ptr);
      t.precisionStart();
      geometry_map_.addMapEntry(map_entry_ptr);
      double step_time =t.precisionStop();
      //ROS_INFO("Adding feature took %f s", step_time);
      time+=step_time;
      //ROS_INFO("[feature map] Accumulated time at step %d: %f s", ctr, time);
      ctr++;
    }
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
    cob_3d_mapping_msgs::ShapeArray map_msg;
    map_msg.header.frame_id="/map";
    map_msg.header.stamp = ros::Time::now();
    map_pub_.publish(map_msg);
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
    boost::shared_ptr<std::vector<PolygonPtr> > map = geometry_map_.getMap();
    res.map.header.stamp = ros::Time::now();
    res.map.header.frame_id = "/map";
    for(unsigned int i=0; i<map->size(); i++)
    {
      Polygon& sm = *(map->at(i));
      cob_3d_mapping_msgs::Shape s;
      toROSMsg(sm,s);
      res.map.shapes.push_back(s);
    }
    return true;
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
  void dumpPolygonToFile(Polygon& m)
  {
    static int ctr=0;
    std::stringstream ss;
    ss << "/home/goa/tmp/polygon_" << ctr << ".txt";
    std::ofstream myfile;
    myfile.open (ss.str().c_str());
    myfile << m.id << "\n";
    myfile << m.normal(0) << "\n" << m.normal(1) << "\n" << m.normal(2) << "\n";
    myfile << m.contours[0].size() << "\n";
    for(unsigned int i=0; i<m.contours.size(); i++)
    {
      for(unsigned int j=0; j<m.contours[i].size(); j++)
      {
        myfile << m.contours[i][j](0) << " ";
        myfile << m.contours[i][j](1) << " ";
        myfile << m.contours[i][j](2) << "\n";
      }
    }

    myfile.close();

    ctr++;
  }


  void publishMap()
  {
    boost::shared_ptr<std::vector<PolygonPtr> > map = geometry_map_.getMap();
    geometry_map_.colorizeMap();
    //cob_3d_mapping_msgs::PolygonArrayArray map_msg;
    cob_3d_mapping_msgs::ShapeArray map_msg;
    map_msg.header.frame_id="/map";
    map_msg.header.stamp = ros::Time::now();
    for(unsigned int i=0; i<map->size(); i++)
    {
      Polygon& sm = *(map->at(i));
      //cob_3d_mapping_msgs::PolygonArray p;
      cob_3d_mapping_msgs::Shape s;
      toROSMsg(sm, s);
      s.header = map_msg.header;
      //s.color.b = 1;
      //s.color.a = 1;
      //map_msg.polygon_array.push_back(p);
      map_msg.shapes.push_back(s);
    }
    map_pub_.publish(map_msg);
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
    visualization_msgs::Marker marker, t_marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration();
    marker.header.frame_id = "/map";

    t_marker.action = visualization_msgs::Marker::ADD;
    t_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    t_marker.lifetime = ros::Duration();
    t_marker.header.frame_id = "/map";
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
    boost::shared_ptr<std::vector<PolygonPtr> > map = geometry_map_.getMap();
    int ctr=0, t_ctr=2000;
    for(unsigned int i=0; i<map->size(); i++)
    {
      Polygon& pm = *(map->at(i));
      int color_ctr = i%4;
      //marker.id = pm.id;
      if(color_ctr==0)
      {
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
      }
      else if(color_ctr==1)
      {
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
      }
      else if(color_ctr==2)
      {
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 1;
      }
      else if(color_ctr==3)
      {
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 0;
      }
      for(unsigned int j=0; j<pm.contours.size(); j++)
      {
        //if(pm.contours.size()>1) std::cout << "id: " << ctr << ", " << pm.contours.size() << std::endl;
        //TODO: this is a workaround as the marker can't display more than one contour
        marker.id = ctr;
        marker.color.r /= j+1;
        marker.color.g /= j+1;
        marker.color.b /= j+1;

        t_marker.id = t_ctr;
        std::stringstream ss;
        ss << ctr;
        t_marker.text = ss.str();
        ctr++;
        t_ctr++;
        for(unsigned int k=0; k<pm.contours[j].size(); k++)
        {
          marker.points.resize(pm.contours[j].size()+1);
          /*pt.x = pm.contours[j][k](0);
                  pt.y = pm.contours[j][k](1);
                  pt.z = pm.contours[j][k](2);*/
          marker.points[k].x = pm.contours[j][k](0);
          marker.points[k].y = pm.contours[j][k](1);
          marker.points[k].z = pm.contours[j][k](2);
          //marker.points.push_back(pt);
        }
        marker.points[pm.contours[j].size()].x = pm.contours[j][0](0);
        marker.points[pm.contours[j].size()].y = pm.contours[j][0](1);
        marker.points[pm.contours[j].size()].z = pm.contours[j][0](2);
        marker_pub_.publish(marker);
        marker_pub_.publish(t_marker);
      }
    }
  }

  ros::NodeHandle n_;


protected:
  ros::Subscriber shape_sub_;
  ros::Publisher map_pub_;
  ros::Publisher marker_pub_;
  ros::ServiceServer clear_map_server_;
  ros::ServiceServer get_map_server_;
  dynamic_reconfigure::Server<cob_3d_mapping_geometry_map::geometry_map_nodeConfig> config_server_;

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

