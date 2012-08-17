/*
 * shape_visualization.h
 *
 *  Created on: Jul 25, 2012
 *      Author: goa-sn
 */

#ifndef SHAPE_VISUALIZATION_H_
#define SHAPE_VISUALIZATION_H_

//##################
//#### includes ####
// standard includes
#include <stdio.h>
#include <sstream>

// ROS includes
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
//#include <visualization_msgs/MenuEntry.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <cob_3d_mapping_msgs/ModifyMap.h>


#include <boost/shared_ptr.hpp>


class ShapeVisualization
{
  public:
    // Constructor
    ShapeVisualization () :ctr_for_shape_indexes (0)

    {
      shape_array_sub_ = nh_.subscribe ("shape_array", 1, &ShapeVisualization::shapeArrayCallback, this);
      //viz_msg_pub_ = nh_.advertise<visualization_msgs::Marker> ("marker",10);
      //marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("marker_array",10);
      //    viz_msg_im_pub_ = nh_.advertise<visualization_msgs::InteractiveMarker> ("interactive_marker", 1);
      //    shape_pub_ = nh_.advertise<cob_3d_mapping_msgs::Shape> ("shape", 1);
      im_server_.reset (new interactive_markers::InteractiveMarkerServer ("geometry_map/map", "", false));
      //std::cout << "Ptr in Vis: " << im_server_.get() << std::endl;
      //    sha = cob_3d_mapping_msgs::ShapeArrayPtr(new cob_3d_mapping_msgs::ShapeArray) ;

    }

    // Destructor
    ~ShapeVisualization ()
    {
      /// void
    }

    void shapeArrayCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& sa) ;
  protected:

      ros::NodeHandle nh_;

      ros::Subscriber shape_array_sub_; // sub for shape array msgs
      std::vector<boost::shared_ptr<ShapeMarker> > v_sm_;
      cob_3d_mapping_msgs::ShapeArray sha ;
      int ctr_for_shape_indexes ;
//      int counter ;


      boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_; // server for interactive markers

};










#endif /* SHAPE_VISUALIZATION_H_ */
