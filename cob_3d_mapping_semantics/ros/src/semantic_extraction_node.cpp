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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_semantics
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer email:Waqas.Tanveer@ipa.fraunhofer.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 11/2011
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


//standard includes
//#include <iostream>

// ROS includes
#include <ros/ros.h>

//ros message includes
#include <cob_3d_mapping_msgs/PolygonArrayArray.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

//internal includes
#include <cob_3d_mapping_semantics/semantic_extraction.h>

class SemanticExtractionNode
{
public:

  // Constructor
  SemanticExtractionNode ()
  {
    poly_sub_ = n_.subscribe ("feature_map", 1, &SemanticExtractionNode::callback, this);
    //poly_pub_ = n_.advertise<cob_3d_mapping_msgs::PolygonArrayArray> ("polygon_array", 1);
    marker_pub_ = n_.advertise<visualization_msgs::Marker> ("polygon_marker", 100);
    pc_pub_ = n_.advertise<sensor_msgs::PointCloud> ("point_cloud", 10);
  }

  // Destructor
  ~SemanticExtractionNode ()
  {
    /// void
  }

  /**
   * @brief callback for publishing new PolygonArrayArray messages
   *
   * @param p ros message containing array of polygon_array messages
   *
   * @return nothing
   */
  void
  callback (const cob_3d_mapping_msgs::PolygonArrayArray::ConstPtr p)
  {
    ROS_INFO("\n\t-------------------------------------\n"
        "\t|       NEW MESSAGE RECEIVED        |\n"
        "\t-------------------------------------\n");

    //cob_3d_mapping_msgs::PolygonArrayArray p_out;

    std::cout << " Total number of polygons:  " << p->polygon_array.size () << std::endl;
    for (unsigned int i = 0; i < p->polygon_array.size (); i++)
    {
      std::cout << " polygon < " << i << " > passed for conversion  " << std::endl;

      //ROS_INFO(" Entered for loop \n");
      SemanticExtraction::PolygonPtr poly_ptr = SemanticExtraction::PolygonPtr (new SemanticExtraction::Polygon ());
      sensor_msgs::PointCloudPtr pc_ptr (new sensor_msgs::PointCloud);

      SemanticExtraction sem_exn;
      bool plane_horizontal, height_ok, size_ok;

      convertFromROSMsg (p->polygon_array[i], *poly_ptr);

      plane_horizontal = sem_exn.isHorizontal (poly_ptr);

      //Check if the plane spanned by the polygon is horizontal or not
      if (plane_horizontal)
      {

        ROS_INFO(" Plane is h0rizontal \n");
        //publishPolygonMarker (*poly_ptr);
        height_ok = sem_exn.isHeightOk (poly_ptr);

        //Check if the height of the polygon is Ok or not
        if (height_ok)
        {
          ROS_INFO(" Height is ok \n");
          //publishPolygonMarker (*poly_ptr);
          size_ok = sem_exn.isSizeOk (poly_ptr);

          //Check if the area of the the polygon is ok or not
          if (size_ok)
          {
            ROS_INFO(" Size is ok \n");
            publishPolygonMarker (*poly_ptr); //publish marker messages to see visually on rviz
            convertToPointCloudMsg (*poly_ptr, *pc_ptr);
            pc_pub_.publish (*pc_ptr);
          }

          else
          {
            ROS_INFO(" Size is not ok \n");
          }

        }

        else
        {
          ROS_INFO(" Height is not ok \n");
        }

      }

      else
      {
        ROS_INFO(" Plane is not horizontal \n");
      }

    }
    //poly_pub_.publish (p_out);

  }

  /**
   * @brief convert ros message to polygon struct
   *
   * @param p ros message to be converted
   *
   * @param poly polygon stuct
   * @return nothing
   */
  void
  convertFromROSMsg (const cob_3d_mapping_msgs::PolygonArray& p, SemanticExtraction::Polygon& poly)
  {
    ROS_INFO(" converting ROS msg \n");

    poly.normal (0) = p.normal.x;
    poly.normal (1) = p.normal.y;
    poly.normal (2) = p.normal.z;
    poly.d = p.d.data;

    for (unsigned int i = 0; i < p.polygons.size (); i++)
    {
      std::cout << "\tpolygon < " << i << " > size: " << p.polygons.size () << std::endl;
      if (p.polygons[i].points.size ())
      {
        std::cout << "\tpoints in polygon < " << i << " >  : " << p.polygons[i].points.size () << std::endl;

        //std::vector<Eigen::Vector3f>  pts;
        //Eigen::Vector3f p3f;

        std::vector<Eigen::Vector3f> pts;
        pts.resize (p.polygons[i].points.size ());

        //std::cout<<" pts size : "<< pts.size()<<std::endl;
        //std::cout<<"\n\n_"<<i<<"_";

        for (unsigned int j = 0; j < p.polygons[i].points.size (); j++)
        {
          //std::cout<<"\n\t#"<<j<<"#";

          pts[j][0] = p.polygons[i].points[j].x;
          pts[j][1] = p.polygons[i].points[j].y;
          pts[j][2] = p.polygons[i].points[j].z;

          //std::cout<<"\t%x = "<< pts[j][0]<<std::endl;
          //std::cout<<"\t\t%y = "<< pts[j][1]<<std::endl;
          //std::cout<<"\t\t%z = "<< pts[j][2]<<std::endl;

          /*
           p3f[0] = p.polygons[i].points[j].x;
           std::cout<<"\t\t%%x = "<< p3f[0]<<std::endl;
           p3f[1] = p.polygons[i].points[j].y;
           std::cout<<"\t\t%%x = "<< p3f[1]<<std::endl;
           p3f[2] = p.polygons[i].points[j].z;
           std::cout<<"\t\t%%x = "<< p3f[2]<<std::endl;

           pts.push_back(p3f);
           std::cout<<" pts point : "<< pts[j]<<std::endl;

           */
        }

        poly.poly_points.push_back (pts);
      }
    }
  }
  /*
   void
   convertToROSMsg (const SemanticExtraction::Polygon& poly, cob_3d_mapping_msgs::PolygonArray& p)
   {
   ROS_INFO(" converting to ROS msg \n");

   p.normal.x = poly.normal (0);
   p.normal.y = poly.normal (1);
   p.normal.z = poly.normal (2);
   p.d.data = poly.d;

   //std::cout<<" -:--------------:"<<std::endl;
   //std::cout<<" normal(0) :"<<p.normal.x<<std::endl;

   //p.polygons.resize(poly.poly_points.size());
   for (unsigned int i = 0; i < poly.poly_points.size (); i++)
   {
   std::cout << " poly size : " << poly.poly_points.size () << std::endl;
   if (poly.poly_points.size ())
   {
   std::cout << " poly at i size: " << poly.poly_points[i].size () << std::endl;

   //std::cout<<"\n\n_"<<i<<"_";

   for (unsigned int j = 0; j < poly.poly_points[i].size (); j++)
   {
   //std::cout<<"\n\t#"<<j<<"#";

   p.polygons[i].points[j].x = poly.poly_points[i][j][0];
   p.polygons[i].points[j].y = poly.poly_points[i][j][1];
   p.polygons[i].points[j].z = poly.poly_points[i][j][2];

   //std::cout<<"\t%x = "<< pts[j][0]<<std::endl;
   //std::cout<<"\t\t%y = "<< pts[j][1]<<std::endl;
   //std::cout<<"\t\t%z = "<< pts[j][2]<<std::endl;
   }

   p.polygons.push_back (p.polygons[i]);
   }
   }

   }
   */
  /**
   * @brief convert polygon to PointCloud message
   *
   * @param poly polygon to be converted
   *
   * @param pc resultant point_cloud message
   * @return nothing
   */
  void
  convertToPointCloudMsg (const SemanticExtraction::Polygon& poly, sensor_msgs::PointCloud& pc)
  {
    pc.header.frame_id = "/map";

    for (unsigned int i = 0; i < poly.poly_points.size (); i++)
    {
      std::cout << " poly size : " << poly.poly_points.size () << std::endl;
      if (poly.poly_points.size ())
      {
        geometry_msgs::Point32 pts;
        for (unsigned int j = 0; j < poly.poly_points[i].size (); j++)
        {
          //std::cout<<"\n\t#"<<j<<"#";
          pts.x = poly.poly_points[i][j][0];
          pts.y = poly.poly_points[i][j][1];
          pts.z = poly.poly_points[i][j][2];
          pc.points.push_back (pts);
        }
      }
    }

  }
  /**
   * @brief publishes markers to visualize polygon in rviz
   *
   * @param poly polygon to be seen visually
   *
   * @return nothing
   */
  void
  publishPolygonMarker (const SemanticExtraction::Polygon& poly)
  {

    std::cout << " polygons size : " << poly.poly_points.size () << std::endl;
    int ctr = 0;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration ();
    marker.header.frame_id = "/map";
    marker.ns = "polygon_marker";
    //marker.header.stamp = ros::Time::now();

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1.0;

    for (unsigned int i = 0; i < poly.poly_points.size (); i++)
    {
      // std::cout<<" polygons in marker at i size : "<< poly.poly_points[i].size ()<<std::endl;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;

      marker.id = ctr;
      ctr++;
      //geometry_msgs::Point pt;
      marker.points.resize (poly.poly_points[i].size () + 1);
      for (unsigned int j = 0; j < poly.poly_points[i].size (); j++)
      {
        /*
         pt.x= poly.poly_points[i][j](0);
         pt.y= poly.poly_points[i][j](0);
         pt.z= poly.poly_points[i][j](0);
         */
        marker.points[j].x = poly.poly_points[i][j] (0);
        marker.points[j].y = poly.poly_points[i][j] (1);
        marker.points[j].z = poly.poly_points[i][j] (2);

        //marker.points.push_back(pt);

      }

      marker.points[poly.poly_points[i].size ()].x = poly.poly_points[i][0] (0);
      marker.points[poly.poly_points[i].size ()].y = poly.poly_points[i][0] (1);
      marker.points[poly.poly_points[i].size ()].z = poly.poly_points[i][0] (2);
      //std::cout<<" ctr : "<<ctr<<std::endl;

      marker_pub_.publish (marker);
    }

  }

  ros::NodeHandle n_;

protected:
  ros::Subscriber poly_sub_;
  //ros::Publisher poly_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher pc_pub_;

};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_extraction_node");

  SemanticExtractionNode sen;
  //ros::spin ();

  ros::Rate loop_rate (10);
  while (ros::ok ())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}
