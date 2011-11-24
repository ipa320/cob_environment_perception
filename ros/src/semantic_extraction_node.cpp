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

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <iostream>
#include <cob_3d_mapping_msgs/PolygonArrayArray.h>

#include <cob_3d_mapping_semantics/semantic_extraction.h>
class SemanticExtractionNode
{
public:

  // Constructor
  SemanticExtractionNode ()
  {
    //ROS_INFO("subscribing to point_cloud messages");
    map_sub_ = n_.subscribe ("feature_map", 1, &SemanticExtractionNode::callback, this);
    //pc_pub_ = n_.advertise<sensor_msgs::PointCloud2>("point_cloud",1);
    pc_pub_ = n_.advertise<cob_3d_mapping_msgs::PolygonArrayArray> ("polygon_array", 1);

  }

  // Destructor
  ~SemanticExtractionNode ()
  {
    /// void
  }

  void

  callback (const cob_3d_mapping_msgs::PolygonArrayArray::ConstPtr p)
  {
    ROS_INFO(" Entered callback method \n");
    SemanticExtraction::PolygonPtr poly_ptr = SemanticExtraction::PolygonPtr(new SemanticExtraction::Polygon());

    cob_3d_mapping_msgs::PolygonArrayArray::ConstPtr p_array = cob_3d_mapping_msgs::PolygonArrayArray::ConstPtr(new cob_3d_mapping_msgs::PolygonArrayArray());

    std::cout<<" polygon_array size:  "<< p->polygon_array.size()<<std::endl;

    for (unsigned int i = 0; i < p->polygon_array.size(); i++)
    {
       ROS_INFO(" Entered for loop \n");

       convertFromROSMsg(p->polygon_array[i], *poly_ptr);
       SemanticExtraction sem_exn;

       bool plane_horizontal, height_ok;
         plane_horizontal = sem_exn.isHorizontal(poly_ptr);
       if(plane_horizontal)
       {
         ROS_INFO(" Plane is h0rizontal \n");
         height_ok = sem_exn.isHeightOk(poly_ptr);

         if(height_ok)
          ROS_INFO(" Height is ok \n");
         else
           ROS_INFO(" Height is not ok \n");

       }
       else
         ROS_INFO(" Plane is not horizontal \n");


//      convertToROSMsg(*poly_ptr, p_array->polygon_array[i]);
    }


  //
  //
   pc_pub_.publish (p_array);

  }

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
      std::cout<<" polygons size : "<< p.polygons.size ()<<std::endl;
      if (p.polygons[i].points.size ())
      {
        std::cout<<" polygons at i size: "<< p.polygons[i].points.size ()<<std::endl;

        //std::vector<Eigen::Vector3f>  pts;
        //Eigen::Vector3f p3f;

        std::vector<Eigen::Vector3f>  pts;
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
  convertToROSMsg (SemanticExtraction::Polygon& poly ,const cob_3d_mapping_msgs::PolygonArray& p)
    {
      ROS_INFO(" converting to ROS msg \n");

      p.normal.x = poly.normal (0);
      p.normal.y = poly.normal (1);
      p.normal.z = poly.normal (2);
      p.d.data = poly.d ;


      //p.polygons.resize(poly.poly_points.size());
      for (unsigned int i = 0; i < poly.poly_points.size(); i++)
      {
        std::cout<<" poly size : "<< poly.poly_points.size()<<std::endl;
        if (poly.poly_points.size())
        {
          std::cout<<" poly at i size: "<< poly.poly_points[i].size()<<std::endl;

         //std::cout<<"\n\n_"<<i<<"_";

          for (unsigned int j = 0; j < poly.poly_points[i].size(); j++)
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
  ros::NodeHandle n_;
/*
  void
  fill_polygon (msg, polygon)
*/
protected:
  ros::Subscriber map_sub_;
  ros::Publisher pc_pub_;

};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_extraction_node");

  SemanticExtractionNode sen;
  ros::spin ();

  ros::Rate loop_rate (10);
  while (ros::ok ())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}
