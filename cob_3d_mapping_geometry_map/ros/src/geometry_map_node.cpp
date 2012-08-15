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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transform.h>
#include <cob_3d_mapping_geometry_map/geometry_map_nodeConfig.h>
#include "pcl/surface/convex_hull.h"
#include "pcl/filters/project_inliers.h"


// external includes
#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <cob_3d_mapping_common/ros_msg_conversions.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


// internal includes
#include "cob_3d_mapping_geometry_map/geometry_map_node.h"

using namespace cob_3d_mapping;

GeometryMapNode::GeometryMapNode()
{
  enable_tf_=true;
  map_frame_id_="/map";
  config_server_.setCallback(boost::bind(&GeometryMapNode::dynReconfCallback, this, _1, _2));
  ctr_ = 0;
  shape_sub_ = n_.subscribe("shape_array", 10, &GeometryMapNode::shapeCallback, this);
  map_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray>("map_array",1);
  marker_pub_ = n_.advertise<visualization_msgs::Marker>("geometry_marker",100);
  clear_map_server_ = n_.advertiseService("clear_map", &GeometryMapNode::clearMap, this);
  get_map_server_ = n_.advertiseService("get_map", &GeometryMapNode::getMap, this);
  ros::param::param("~file_path" , file_path_ , std::string("/home/goa-tz/tmp/"));
  ros::param::param("~save_to_file" , save_to_file_ , false);
  //ros::param::param("~map_frame_id", map_frame_id_, "/map");
  std::cout << file_path_ << std::endl;
  geometry_map_.setFilePath(file_path_);
  geometry_map_.setSaveToFile(save_to_file_);
}

void
GeometryMapNode::dynReconfCallback(cob_3d_mapping_geometry_map::geometry_map_nodeConfig &config, uint32_t level)
{
  ROS_INFO("[geometry_map]: received new parameters");
  geometry_map_.setSaveToFile( config.save_to_file );
  geometry_map_.setMergeThresholds(config.cos_angle, config.d);
  map_frame_id_ = config.map_frame_id;
  enable_tf_ = config.enable_tf;
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
GeometryMapNode::shapeCallback(const cob_3d_mapping_msgs::ShapeArray::ConstPtr sa)
{
  tf::StampedTransform trf_map;
  Eigen::Affine3f af_orig = Eigen::Affine3f::Identity();

  try
  {
    tf_listener_.waitForTransform(map_frame_id_, sa->header.frame_id, sa->header.stamp, ros::Duration(2));
    tf_listener_.lookupTransform(map_frame_id_, sa->header.frame_id, sa->header.stamp, trf_map);
  }
  catch (tf::TransformException ex) { ROS_ERROR("[geometry map node] : %s",ex.what()); return; }

  Eigen::Affine3d ad;
  tf::TransformTFToEigen(trf_map, ad);
  af_orig = ad.cast<float>();
  af_orig = geometry_map_.getLastError() * af_orig;

  static int ctr=0;
  static double time = 0;
  PrecisionStopWatch t;
  std::cout<<">>>>>>>>>new cloud>>>>>>>>>>\n";

  std::vector<Polygon::Ptr> polygon_list;
  std::vector<CylinderPtr> cylinder_list;

  for(size_t i=0; i<sa->shapes.size(); ++i)
  {
    switch (sa->shapes[i].type)
    {
    case cob_3d_mapping_msgs::Shape::POLYGON:
    {
      polygon_list.push_back(Polygon::Ptr(new Polygon));
      fromROSMsg(sa->shapes[i], *polygon_list.back());
      polygon_list.back()->transform2tf(af_orig);
      break;
    }
    case cob_3d_mapping_msgs::Shape::CYLINDER:
    {
      cylinder_list.push_back(Cylinder::Ptr(new Cylinder));
      fromROSMsg(sa->shapes[i], *cylinder_list.back());
      cylinder_list.back()->transform2tf(af_orig);
      break;
    }
    default:
      break;
    }
  }

  Eigen::Affine3f af_new;
  // currently turned off, always returns false
  bool needs_adjustment = geometry_map_.computeTfError(polygon_list, af_orig, af_new);

  for (size_t i=0; i<polygon_list.size(); ++i)
  {
    //Eigen::Vector3f n = polygon_list[i]->normal;
    //std::cout<<"n:"<<n(0)<<","<<n(1)<<","<<n(2)<<" before"<<std::endl;
    if(needs_adjustment) polygon_list[i]->transform2tf(af_new);
    //n = polygon_list[i]->normal;
    //std::cout<<"n:"<<n(0)<<","<<n(1)<<","<<n(2)<<" after"<<std::endl;
    geometry_map_.addMapEntry(polygon_list[i]);
  }
  for (size_t i=0; i<cylinder_list.size(); ++i)
  {
    if(needs_adjustment) cylinder_list[i]->transform2tf(af_new);
    geometry_map_.addMapEntry(cylinder_list[i]);
  }

  geometry_map_.cleanUp();
  geometry_map_.incrFrame();

  publishMapMarker();
  publishMap();
  ctr_++;
}

bool
GeometryMapNode::clearMap(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
{
  //TODO: add mutex
  ROS_INFO("Clearing geometry map...");
  geometry_map_.clearMap();
  cob_3d_mapping_msgs::ShapeArray map_msg;
  map_msg.header.frame_id=map_frame_id_;
  map_msg.header.stamp = ros::Time::now();
  map_pub_.publish(map_msg);
  return true;
}


bool
GeometryMapNode::getMap(cob_3d_mapping_msgs::GetGeometricMap::Request &req, cob_3d_mapping_msgs::GetGeometricMap::Response &res)
{
  boost::shared_ptr<std::vector<Polygon::Ptr> > map_polygon = geometry_map_.getMap_polygon();
  boost::shared_ptr<std::vector<CylinderPtr> > map_cylinder = geometry_map_.getMap_cylinder();

  res.map.header.stamp = ros::Time::now();
  res.map.header.frame_id = map_frame_id_;
  for(unsigned int i=0; i<map_polygon->size(); i++)
  {
    Polygon& sm = *(map_polygon->at(i));
    cob_3d_mapping_msgs::Shape s;
    toROSMsg(sm,s);
    res.map.shapes.push_back(s);
  }

  for(unsigned int i=0; i<map_cylinder->size(); i++)
  {
    Cylinder& sm = *(map_cylinder->at(i));
    cob_3d_mapping_msgs::Shape s;
    toROSMsg(sm,s);
    res.map.shapes.push_back(s);
  }

  return true;
}

void
GeometryMapNode::dumpPolygonToFile(Polygon& m)
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

void
GeometryMapNode::publishMap()
{
  //		if index = type 1 poly ptr , else cylinder ptr --> push back in shape vector?!

  boost::shared_ptr<std::vector<Polygon::Ptr> > map_polygon = geometry_map_.getMap_polygon();
  boost::shared_ptr<std::vector<CylinderPtr> > map_cylinder = geometry_map_.getMap_cylinder();


  geometry_map_.colorizeMap();
  //cob_3d_mapping_msgs::PolygonArrayArray map_msg;
  cob_3d_mapping_msgs::ShapeArray map_msg;
  map_msg.header.frame_id=map_frame_id_;
  map_msg.header.stamp = ros::Time::now();

  //		std::cout<<"_________________________________"<<std::endl;
  //		std::cout<<"polygon size: "<<map_polygon->size()<<std::endl;
  //		polygons
  for(unsigned int i=0; i<map_polygon->size(); i++)
  {
    Polygon& sm = *(map_polygon->at(i));
    //			std::cout<<sm.d<<std::endl<<std::endl;
    //cob_3d_mapping_msgs::PolygonArray p;
    cob_3d_mapping_msgs::Shape s;
    toROSMsg(sm, s);
    s.header = map_msg.header;
    //s.color.b = 1;
    //s.color.a = 1;
    //map_msg.polygon_array.push_back(p);
    map_msg.shapes.push_back(s);
  }

  //		cylinders
  for(unsigned int i=0; i<map_cylinder->size(); i++)
  {
    Cylinder& sm = *(map_cylinder->at(i));
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

void
GeometryMapNode::publishMapMarker()
{
  visualization_msgs::Marker marker, t_marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration();
  marker.header.frame_id = map_frame_id_;

  t_marker.action = visualization_msgs::Marker::ADD;
  t_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  t_marker.lifetime = ros::Duration();
  t_marker.header.frame_id = map_frame_id_;
  //marker.header.stamp = stamp;

  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 1;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1.0;

  geometry_msgs::Point pt;

  //		only implemented for polygon

  boost::shared_ptr<std::vector<Polygon::Ptr> > map_polygon = geometry_map_.getMap_polygon();

  int ctr=0, t_ctr=2000;
  //		std::cout<<"____________________________________________"<<std::endl;
  //		std::cout<<"marker size: "<<map->size()<<std::endl;
  for(unsigned int i=0; i<map_polygon->size(); i++)
  {
    Polygon& pm = *(map_polygon->at(i));
    int color_ctr = i%5;
    //marker.id = pm.id;
    switch(color_ctr)
    {
    case 0:
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      break;
    case 1:
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
      break;
    case 2:
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 1;
      break;
    case 3:
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 0;
      break;
    case 4:
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 1;
      break;
    }

    //			std::cout<<pm.d<<std::endl<<std::endl;

    for(unsigned int j=0; j<pm.contours.size(); j++)
    {
      //if(pm.contours.size()>1) std::cout << "id: " << ctr << ", " << pm.contours.size() << std::endl;
      //TODO: this is a workaround as the marker can't display more than one contour
      marker.id = ctr;
      if (pm.holes[j])
      {
        marker.color.r /= static_cast<float>(j+2);
        marker.color.g /= static_cast<float>(j+2);
        marker.color.b /= static_cast<float>(j+2);
      }

      t_marker.id = t_ctr;
      std::stringstream ss;
      ss << ctr;
      t_marker.text = ss.str();
      ctr++;
      t_ctr++;

      marker.points.resize(pm.contours[j].size()+1);
      for(unsigned int k=0; k<pm.contours[j].size(); k++)
      {
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
  //		only implemented for polygon

  boost::shared_ptr<std::vector<CylinderPtr> > map_cylinder = geometry_map_.getMap_cylinder();

  ctr=0;
  t_ctr=2000;

  //		std::cout<<"____________________________________________"<<std::endl;
  //		std::cout<<"marker size: "<<map->size()<<std::endl;
  for(unsigned int i=0; i<map_cylinder->size(); i++)
  {
    Cylinder& cm = *(map_cylinder->at(i));
    int color_ctr = i%4;
    //marker.id = cm.id;
    marker.color.r=1;
    marker.color.g=0;
    marker.color.b=0;
    //					if(color_ctr==0)
    //					{
    //						marker.color.r = 0;
    //						marker.color.g = 0;
    //						marker.color.b = 1;
    //					}
    //					else if(color_ctr==1)
    //					{
    //						marker.color.r = 0;
    //						marker.color.g = 1;
    //						marker.color.b = 0;
    //					}
    //					else if(color_ctr==2)
    //					{
    //						marker.color.r = 0;
    //						marker.color.g = 1;
    //						marker.color.b = 1;
    //					}
    //					else if(color_ctr==3)
    //					{
    //						marker.color.r = 1;
    //						marker.color.g = 1;
    //						marker.color.b = 0;
    //					}


    //			std::cout<<pm.d<<std::endl<<std::endl;

    //					get 3dimensional contours
    std::vector<std::vector<Eigen::Vector3f> > contours3d;
    cm.getCyl3D(contours3d);

    for(unsigned int j=0; j<contours3d.size(); j++)
    {
      //if(pm.contours.size()>1) std::cout << "id: " << ctr << ", " << pm.contours.size() << std::endl;
      //TODO: this is a workaround as the marker can't display more than one contour
      marker.id = ctr;
      //						marker.color.r /= j+1;
      //						marker.color.g /= j+1;
      //						marker.color.b /= j+1;
      marker.color.r=1;
      marker.color.g=0;
      marker.color.b=0;

      t_marker.id = t_ctr;
      std::stringstream ss;
      ss << ctr;
      t_marker.text = ss.str();
      ctr++;
      t_ctr++;

      for(unsigned int k=0; k<contours3d[j].size(); k++)
      {
        marker.points.resize(contours3d[j].size()+1);
        /*pt.x = contours3d[j][k](0);
          pt.y = pm.contours[j][k](1);
          pt.z = pm.contours[j][k](2);*/
        marker.points[k].x = contours3d[j][k](0);
        marker.points[k].y = contours3d[j][k](1);
        marker.points[k].z = contours3d[j][k](2);
        //marker.points.push_back(pt);
      }
      marker.points[contours3d[j].size()].x = contours3d[j][0](0);
      marker.points[contours3d[j].size()].y = contours3d[j][0](1);
      marker.points[contours3d[j].size()].z = contours3d[j][0](2);
      marker_pub_.publish(marker);
      marker_pub_.publish(t_marker);

    }
  }
}

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

