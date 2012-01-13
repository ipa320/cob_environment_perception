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
 * ROS parameters for parameters in semantic_extraction class
 * switch to ShapeArray message type for map
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
#include <rosbag/bag.h>

// ros message includes
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_3d_mapping_msgs/PolygonArrayArray.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>

//internal includes
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/GetGeometricMap.h>
#include <cob_3d_mapping_msgs/GetObjectsOfClass.h>
#include <cob_3d_mapping_semantics/semantic_extraction.h>

class SemanticExtractionNode
{
public:

  // Constructor
  SemanticExtractionNode () :
    /*norm_x_min_ (-0.1), norm_x_max_ (0.1), norm_y_min_ (-0.1), norm_y_max_ (0.1), norm_z_min_ (-0.99),
     norm_z_max_ (0.99)*/tilt_angle_ (3.0), height_min_ (0.4), height_max_ (1), area_min_ (1), area_max_ (3)
  {
    sa_sub_ = n_.subscribe ("shape_array", 10, &SemanticExtractionNode::callbackShapeArray, this);
    sa_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray> ("shape_array_pub", 10);
    //pc2_pub_ = n_.advertise<sensor_msgs::PointCloud2> ("shape_pointcloud2", 1);

    s_marker_pub_ = n_.advertise<visualization_msgs::Marker> ("marker", 100);
    map_marker_pub_ = n_.advertise<visualization_msgs::MarkerArray> ("marker_array", 10);

    get_tables_server_ = n_.advertiseService ("get_objects_of_class", &SemanticExtractionNode::getTablesService, this);

    n_.getParam ("semantic_extraction/tilt_angle", tilt_angle_);
    /*
     n_.getParam ("semantic_extraction/norm_x_min", norm_x_min_);
     n_.getParam ("semantic_extraction/norm_x_max", norm_x_max_);
     n_.getParam ("semantic_extraction/norm_y_min", norm_y_min_);
     n_.getParam ("semantic_extraction/norm_y_max", norm_y_max_);
     n_.getParam ("semantic_extraction/norm_z_min", norm_z_min_);
     n_.getParam ("semantic_extraction/norm_z_max", norm_z_max_);
     */
    n_.getParam ("semantic_extraction/height_min", height_min_);
    n_.getParam ("semantic_extraction/height_max", height_max_);

    n_.getParam ("semantic_extraction/area_min", area_min_);
    n_.getParam ("semantic_extraction/area_max", area_max_);

    //     ROS PARAMETERS
    /*
     std::cout << "\n\t*norm_x_min = " << norm_x_min_ << std::endl;
     std::cout << "\n\t*norm_x_max = " << norm_x_max_ << std::endl;
     std::cout << "\n\t*norm_y_min = " << norm_y_min_ << std::endl;
     std::cout << "\n\t*norm_y_max = " << norm_y_max_ << std::endl;
     std::cout << "\n\t*norm_z_min = " << norm_z_min_ << std::endl;
     std::cout << "\n\t*norm_z_max = " << norm_z_max_ << std::endl;
     std::cout << "\n\t*height_min = " << height_min_ << std::endl;
     std::cout << "\n\t*height_max = " << height_max_ << std::endl;
     std::cout << "\n\t*area_min = " << area_min_ << std::endl;
     std::cout << "\n\t*area_min = " << area_max_ << std::endl;


     sem_exn_.setNormXMin (norm_x_min_);
     sem_exn_.setNormXMax (norm_x_max_);
     sem_exn_.setNormYMin (norm_y_min_);
     sem_exn_.setNormYMax (norm_y_max_);
     sem_exn_.setNormZMin (norm_z_min_);
     sem_exn_.setNormZMax (norm_z_max_);
     */
    sem_exn_.setNormBounds (tilt_angle_);
    sem_exn_.setHightMin (height_min_);
    sem_exn_.setHightMax (height_max_);
    sem_exn_.setAreaMin (area_min_);
    sem_exn_.setAreaMax (area_max_);

  }

  // Destructor
  ~SemanticExtractionNode ()
  {
    /// void
  }

  //TODO: get map also by service
  /**
   * @brief callback for ShapeArray messages
   *
   * @param sa_ptr pointer to the message
   *
   * @return nothing
   */

  void
  callbackShapeArray (const cob_3d_mapping_msgs::ShapeArray::ConstPtr sa_ptr)
  {

    ROS_INFO( "\n\t-------------------------------------\n"
        "\t|       NEW MESSAGE RECEIVED        |\n"
        "\t-------------------------------------\n");

    ROS_INFO(" Total number of shapes received: %d ", sa_ptr->shapes.size());

    cob_3d_mapping_msgs::ShapeArray sa_msg;
    sa_msg.header.frame_id = "/map";
    //sem_exn_.PolygonMap.resize (0);
    for (unsigned int i = 0; i < sa_ptr->shapes.size (); i++)
    {

      ROS_INFO("Shape < %d > passed for conversion ",i);

      //ROS_INFO(" Entered for loop \n");
      SemanticExtraction::PolygonPtr poly_ptr = SemanticExtraction::PolygonPtr (new SemanticExtraction::Polygon ());

      convertShapeToPolygon (sa_ptr->shapes[i], *poly_ptr);
      //sem_exn_.PolygonMap.push_back (poly_ptr);
      ROS_INFO("sa_ptr->shapes[%d] converted to polygon",i );
      //convertFromROSMsg (p->polygon_array[i], *poly_ptr);
      //convertToPointCloudMsg (*poly_ptr, *pc_ptr);
      //pc_pub_.publish (*pc_ptr);
      if (isTableObject (poly_ptr) == true)
      {
        //ros::Duration (5).sleep ();
        cob_3d_mapping_msgs::Shape s;
        convertPolygonToShape (*poly_ptr, s);
        //publishShapeMarker (s);
        sa_msg.shapes.push_back (s);
      }

      /*
       cob_3d_mapping_msgs::Shape s;

       //Check if the plane spanned by the polygon is horizontal or not
       if (sem_exn_.isHorizontal (poly_ptr))
       {
       ROS_INFO(" Plane is h0rizontal \n");
       publishPolygonMarker (*poly_ptr);

       sem_exn_.isHeightOk (poly_ptr);
       sem_exn_.isSizeOk (poly_ptr);
       for (unsigned int j = 0; j < sem_exn_.poly_height_.size (); j++)
       {
       pc_index_.push_back(sem_exn_.poly_height_[j] && sem_exn_.poly_size_[j]);

       //Check if the height of the polygon is Ok or not
       if (sem_exn_.poly_height_[j])
       {
       ROS_INFO(" Height is ok \n");
       //Check if the area of the the polygon is ok or not
       if (sem_exn_.poly_size_[j])
       {
       ROS_INFO(" Size is ok \n");
       //publishPolygonMarker (*poly_ptr); //publish marker messages to see visually on rviz
       //TODO: publish as ShapeArray, not as PointCloud
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
       }
       else
       {
       ROS_INFO(" Plane is not horizontal \n");
       }

       convertPolygonToShape (*poly_ptr, s);
       sa_msg.shapes.push_back (s);
       */
    }//end for

    sa_pub_.publish (sa_msg);
    //publishMapMarker ();
  }

  /**
   * @brief check if the polygon is a table object candidate or not
   *
   * @param poly_ptr pointer to the polygon
   *
   * @return true or false
   */
  bool
  isTableObject (SemanticExtraction::PolygonPtr poly_ptr)
  {
    bool flag_horizontal = false;
    pc_index_.resize (0);
    //Check if the plane spanned by the polygon is horizontal or not
    if (sem_exn_.isHorizontal (poly_ptr))
    {
      flag_horizontal = true;
      ROS_INFO(" Plane is h0rizontal \n");
      //publishPolygonMarker (*poly_ptr);

      sem_exn_.isHeightOk (poly_ptr);
      sem_exn_.isSizeOk (poly_ptr);
      for (unsigned int i = 0; i < sem_exn_.poly_height_.size (); i++)
      {
        pc_index_.push_back (sem_exn_.poly_height_[i] && sem_exn_.poly_size_[i]);
      }

      for (unsigned int j = 0; j < pc_index_.size (); j++)
      {
        if (flag_horizontal && pc_index_[j])
        {
          ROS_INFO("<___Table Object___>");
          return true;
          break;
        }
        else
          continue;
      }
      return false;
    }
    else
      return false;

  }

  /**
   * @brief service offering table object candidates
   *
   * @param req request for objects of a class (table objects in this case)
   * @param res response of the service which is possible table object candidates
   *
   * @return true if service successful
   */
  bool
  getTablesService (cob_3d_mapping_msgs::GetObjectsOfClassRequest &req,
                    cob_3d_mapping_msgs::GetObjectsOfClassResponse &res)
  {
    ROS_INFO("service get_objects_of_class started....");

    if (getMapService ())
    {
      sem_exn_.setNormBounds (tilt_angle_);
      for (unsigned int i = 0; i < sem_exn_.PolygonMap.size (); i++)
      {
        //ROS_INFO("\n\tisTableObject....  : ");
        if (isTableObject (sem_exn_.PolygonMap[i]))
        {
          //ros::Duration (10).sleep ();
          cob_3d_mapping_msgs::Shape s;
          convertPolygonToShape (*sem_exn_.PolygonMap[i], s);
          publishShapeMarker (s);
          ROS_INFO("getTablesService: Polygon[%d] converted to shape",i);
          res.objects.shapes.push_back (s);
        }

        /*
         if (sem_exn_.isHorizontal (sem_exn_.PolygonMap[i]))
         {
         ROS_INFO(" Plane is h0rizontal \n");

         //Check if the height of the polygon is Ok or not
         sem_exn_.isHeightOk (sem_exn_.PolygonMap[i]);

         for (unsigned int j = 0; j < sem_exn_.poly_height_.size (); j++)
         {
         if (sem_exn_.poly_height_[j])
         {
         ROS_INFO(" Height is ok \n");
         sem_exn_.isSizeOk (sem_exn_.PolygonMap[i]);
         //Check if the area of the the polygon is ok or not
         if (sem_exn_.poly_size_[j])
         {
         ROS_INFO(" Size is ok \n");
         //publishPolygonMarker (*poly_ptr); //publish marker messages to see visually on rviz
         //TODO: publish as ShapeArray, not as PointCloud
         //convertToPointCloudMsg (*poly_ptr, *pc_ptr);
         //pc_pub_.publish (*pc_ptr);
         cob_3d_mapping_msgs::Shape s;
         convertPolygonToShape (*sem_exn_.PolygonMap[i], s);
         res.objects.shapes.push_back (s);
         sa_.shapes.push_back (s);
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

         }

         else
         {
         ROS_INFO(" Plane is not horizontal \n");
         }

         */
      }
    }
    return true;
  }

  /**
   * @brief service offering geometry map of the scene
   *
   * @return true if service successful
   */
  bool
  getMapService ()
  {
    ROS_INFO("Waiting for service server to start.");
    ros::service::waitForService ("get_geometry_map", 10); //will wait for infinite time

    ROS_INFO("Server started, polling map.");

    //build message
    cob_3d_mapping_msgs::GetGeometricMapRequest req;
    cob_3d_mapping_msgs::GetGeometricMapResponse res;

    if (ros::service::call ("/geometry_map/get_geometry_map", req, res))
    {
      ROS_INFO("Service call finished.");
    }
    else
    {
      ROS_INFO("Service call failed.");
      return 0;
    }
    sem_exn_.PolygonMap.resize (0);
    ROS_INFO(" res.map.shapes.size ()  = %d \n",res.map.shapes.size ());
    for (unsigned int i = 0; i < res.map.shapes.size (); i++)
    {
      SemanticExtraction::PolygonPtr poly_ptr = SemanticExtraction::PolygonPtr (new SemanticExtraction::Polygon ());
      cob_3d_mapping_msgs::Shape& s = res.map.shapes[i];
      SemanticExtraction::Polygon p;
      convertShapeToPolygon (s, *poly_ptr);
      sem_exn_.PolygonMap.push_back (poly_ptr);
    }
    ROS_INFO(" sem_exn_.PolygonMap.size ()  = %d \n",sem_exn_.PolygonMap.size ());
    publishMapMarker ();
    return true;
  }

  /**
   * @brief convert shape to a polygon
   *
   * @param s shape to be converted to polygon
   * @param poly resultant polygon
   *
   * @return nothing
   */
  void
  convertShapeToPolygon (const cob_3d_mapping_msgs::Shape& s, SemanticExtraction::Polygon& poly)
  {
    if (s.type != 0)
      ROS_WARN(" Shape not a plane.");

    else
    {
      ROS_INFO(" converting shape to polygon \n");

      poly.normal (0) = s.params[0];
      poly.normal (1) = s.params[1];
      poly.normal (2) = s.params[2];
      poly.d = s.params[3];

      ROS_INFO(" s.points.size  = %d \n",s.points.size ());
      for (unsigned int i = 0; i < s.points.size (); i++)
      {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        int cloud_size = s.points[i].height * s.points[i].width;

        // ROS_INFO(" cloud_size = %d \n",cloud_size);
        //ROS_INFO(" s.points[i].height = %d \n",s.points[i].height);
        //ROS_INFO(" s.points[i].width = %d \n",s.points[i].width);

        //cloud.points.resize (cloud_size);
        //ROS_INFO("  cloud.points.size = %d \n", cloud.points.size());
        pcl::fromROSMsg (s.points[i], cloud);
        // ROS_INFO("s.points[%d] converted to pcl::PointCloud<pcl::PointXYZ>" ,i);
        /*
         while(1)
         {
         int k = 0;
         while(k<s.points.size ())
         {
         sensor_msgs::PointCloud2 pc2 = s.points[k];
         pc2.header.frame_id = "/map";
         pc2_pub_.publish(pc2);
         k++;
         }

         }
         */

        std::vector<Eigen::Vector3f> p;
        p.resize (cloud_size);
        for (int j = 0; j < cloud_size; j++)
        {
          p[j][0] = cloud.points[j].x;
          p[j][1] = cloud.points[j].y;
          p[j][2] = cloud.points[j].z;
        }
        //ROS_INFO("Poly pushed back ");
        poly.poly_points.push_back (p);
      }
    }

  }
  /**
   * @brief convert polygon to a shape
   *
   * @param poly polygon to be converted to shape
   * @param s resultant shape
   *
   * @return nothing
   */
  void
  convertPolygonToShape (const SemanticExtraction::Polygon& poly, cob_3d_mapping_msgs::Shape& s)
  {
    ROS_INFO(" converting polygon to Shape\n");
    ROS_INFO(" poly.poly_points.size () = %d \n",poly.poly_points.size ());

    //s.params.resize (7);
    s.params.resize (4);

    s.type = 0;
    s.params[0] = poly.normal (0);
    s.params[1] = poly.normal (1);
    s.params[2] = poly.normal (2);
    s.params[3] = poly.d;
    /*
     ROS_INFO("Centroid: %f,%f,%f",poly.centroid[0].x,poly.centroid[0].y,poly.centroid[0].z);
     s.params[4] = poly.centroid[0].x;
     s.params[5] = poly.centroid[0].y;
     s.params[6] = poly.centroid[0].z;
     */
    s.points.resize (0);
    for (unsigned int i = 0; i < poly.poly_points.size (); i++)
    {
      if (pc_index_[i] == true)
      {
        ROS_INFO(" poly.poly_points[i].size () = %d \n",poly.poly_points[i].size ());
        sensor_msgs::PointCloud2 pc2;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        pc2.height = 1;
        pc2.width = poly.poly_points[i].size ();
        for (unsigned int j = 0; j < poly.poly_points[i].size (); j++)
        {
          pcl::PointXYZ pt;

          pt.x = poly.poly_points[i][j][0];
          pt.y = poly.poly_points[i][j][1];
          pt.z = poly.poly_points[i][j][2];

          cloud.points.push_back (pt);
        }

        pcl::toROSMsg (cloud, pc2);
        s.points.push_back (pc2);
        /* pc2.header.frame_id = "/map";
         int k = 0;
         while (k < 1000000)
         {
         pc2_pub_.publish (pc2);
         k++;
         }
         */
      }
    }

  }

  /**
   * @brief publish shape_array messages
   */
  /*  void
   publishShapes ()
   {
   sa_pub_.publish (sa_);
   }
   */
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
      ROS_INFO("polygon < %d > size: %d" , i,p.polygons.size ());
      if (p.polygons[i].points.size ())
      {
        ROS_INFO("polygon < %d > size: %d" , i, p.polygons[i].points.size ());

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
      //std::cout << " poly size : " << poly.poly_points.size () << std::endl;
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
   * @brief publishe markers to visualize shape in rviz
   *
   * @param s shape to be seen visually
   *
   * @return nothing
   */
  void
  publishShapeMarker (const cob_3d_mapping_msgs::Shape& s)
  {
    int ctr = 0;
    //TODO: set unique ID for each new marker, remove old markers


    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration (120);
    marker.header.frame_id = "/map";
    marker.ns = "shape_marker";
    marker.header.stamp = ros::Time::now ();

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    for (unsigned int i = 0; i < s.points.size (); i++)
    {
      marker.id = ctr;
      ctr++;

      // sensor_msgs::PointCloud2 pc2;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      int cloud_size = s.points[i].height * s.points[i].width;

      pcl::fromROSMsg (s.points[i], cloud);

      marker.points.resize (cloud_size);
      for (int j = 0; j < cloud_size; j++)
      {

        marker.points[j].x = cloud[j].x;
        marker.points[j].y = cloud[j].y;
        marker.points[j].z = cloud[j].z;
      }
      /*
       marker.points[poly.poly_points[i].size ()].x = poly.poly_points[i][0] (0);
       marker.points[poly.poly_points[i].size ()].y = poly.poly_points[i][0] (1);
       marker.points[poly.poly_points[i].size ()].z = poly.poly_points[i][0] (2);
       //std::cout<<" ctr : "<<ctr<<std::endl;
       */

      s_marker_pub_.publish (marker);
    }

  }

  /**
   * @brief publish complete map of polygons
   *
   * @return nothing
   */
  void
  publishMapMarker ()
  {

    int ctr = 0;
    visualization_msgs::MarkerArray marker_arr;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration ();
    marker.header.frame_id = "/map";
    marker.ns = "map_marker_array";
    //marker.header.stamp = ros::Time::now();

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1.0;
    //marker_arr.markers.resize (sem_exn_.PolygonMap.size ());
    for (unsigned int k = 0; k < sem_exn_.PolygonMap.size (); k++)
    {

      for (unsigned int i = 0; i < sem_exn_.PolygonMap[k]->poly_points.size (); i++)
      {
        if (ctr % 2 == 0)
        {
          marker.color.r = 0;
          marker.color.g = 0;
          marker.color.b = 1;
        }
        else
        {
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 0;
        }

        marker.id = ctr;
        //ROS_INFO("marker.id = %d",marker.id);
        ctr++;
        marker.points.resize (sem_exn_.PolygonMap[k]->poly_points[i].size ());
        for (unsigned int j = 0; j < marker.points.size (); j++)
        {

          marker.points[j].x = sem_exn_.PolygonMap[k]->poly_points[i][j] (0);
          marker.points[j].y = sem_exn_.PolygonMap[k]->poly_points[i][j] (1);
          marker.points[j].z = sem_exn_.PolygonMap[k]->poly_points[i][j] (2);

          //marker.points.push_back(pt);
        }
        marker_arr.markers.push_back (marker);
      }
      map_marker_pub_.publish (marker_arr);
    }

  }

  ros::NodeHandle n_;

protected:
  ros::Subscriber sa_sub_;
  ros::Publisher sa_pub_;
  ros::Publisher pc2_pub_;
  ros::Publisher s_marker_pub_;
  ros::Publisher map_marker_pub_;
  ros::ServiceServer get_tables_server_;

  SemanticExtraction sem_exn_;

  std::vector<bool> pc_index_;

  double tilt_angle_;
  /*
   double norm_x_min_, norm_x_max_;
   double norm_y_min_, norm_y_max_;
   double norm_z_min_, norm_z_max_;
   */
  double height_min_, height_max_;

  double area_min_, area_max_;
};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_extraction_node");

  SemanticExtractionNode sem_exn_node;
  //ros::spin ();

  ros::Rate loop_rate (10);
  while (ros::ok ())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}
