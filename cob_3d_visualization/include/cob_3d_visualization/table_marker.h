/*
 * table_marker.h
 *
 *  Created on: Sep 25, 2012
 *      Author: goa-sn
 */

#ifndef TABLE_MARKER_H_
#define TABLE_MARKER_H_

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
#include <visualization_msgs/InteractiveMarkerFeedback.h>
//#include <visualization_msgs/MenuEntry.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// PCL includes
#include <pcl/pcl_config.h>
#ifdef PCL_VERSION_COMPARE
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>

//#include <boost/bind.hpp>
//#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

// external includes
#include <Eigen/Core>

//#include <cob_3d_visualization/polypartition.h>
//#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_visualization/shape_marker.h>
//#include <cob_3d_mapping_msgs/ModifyMap.h>

#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"




class TableMarker
{
  public:
    // Constructor
    TableMarker (boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,cob_3d_mapping_msgs::Shape& table,int ctr)
    //: ctr_(ctr)
    {
      id_ = ctr ;
      table_im_server_ = server ;
      table_ = table ;
      createInteractiveMarkerForTable();

    }
    // Destructor
    ~TableMarker ()
    {

    }

    void createMarkerforTable (list<TPPLPoly>& triangle_list,visualization_msgs::InteractiveMarkerControl& im_ctrl);
    void createInteractiveMarkerForTable();
    TPPLPoint msgToPoint2DforTable (const pcl::PointXYZ &point);
    void tableFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  protected:
    visualization_msgs::InteractiveMarker table_int_marker_ ;
    visualization_msgs::Marker table_marker_;

    ros::NodeHandle nh_;

    cob_3d_mapping_msgs::Shape table_;

    visualization_msgs::InteractiveMarkerControl im_ctrl;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> table_im_server_;
    interactive_markers::MenuHandler table_menu_handler_;

    Eigen::Affine3f transformation_;
    Eigen::Affine3f transformation_inv_;

    int id_;
};


#endif /* TABLE_MARKER_H_ */
