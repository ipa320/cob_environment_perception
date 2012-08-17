/*
 * shape_marker.h
 *
 *  Created on: Jul 25, 2012
 *      Author: goa-sn
 */

#ifndef SHAPE_MARKER_H_
#define SHAPE_MARKER_H_


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

//#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_visualization/polypartition.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"

//#define PI 3.14159265

//using namespace cob_3d_mapping ;
class ShapeMarker
{
  public:

    ShapeMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server,
        cob_3d_mapping_msgs::Shape& shape, int ctr, cob_3d_mapping_msgs::ShapeArray ShapeArr)
    {
      sa_ = ShapeArr;
      shape_ctr_ = ctr;
      im_server_ = im_server;
      shape_ = shape;
      createShapeMenu ();
      createInteractiveMarker();
      MoreOptions();
//      shape_indexes.resize(100) ;//(ShapeArr.shapes.size()) ;
      display_arrow = 0 ;
//      ctr_for_shape_indexes = 0 ;
    }

    ~ShapeMarker()
    {
      if(im_server_->erase(marker_.name)){
//        ROS_INFO("Marker %s erased",marker_.name.c_str());
      stringstream ss;
      ss << "normal_" << shape_.id;
      im_server_->erase(ss.str());
      ss.str("");
      ss.clear();
      ss << "centroid_" << shape_.id;
      im_server_->erase(ss.str());
      }
    }

    void MoreOptions();
    void DisplayAllNormals(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void DisplayAllCentroids (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void MoveMarker(int flag) ;//,const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void GetPosition (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    double RandomValue (double min, double max) ;


    void createShapeMenu () ;
    void OptionMenu() ;
    void DeleteMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void displayContour(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    TPPLPoint MsgToPoint2D (const pcl::PointXYZ &point) ;

    void createMarker (list<TPPLPoly>& triangle_list,visualization_msgs::InteractiveMarkerControl& im_ctrl);
    void createInteractiveMarker () ;

    void displayNormalCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void displayCentroidCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;
    void Reset(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) ;




  protected:
    visualization_msgs::InteractiveMarker marker_ ;
    visualization_msgs::InteractiveMarker Imarker ;
    visualization_msgs::Marker marker;


    visualization_msgs::InteractiveMarkerControl im_ctrl;
//    visualization_msgs::InteractiveMarkerControl Im_ctrl;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
//    interactive_markers::InteractiveMarkerServer im_server_ ;
    cob_3d_mapping_msgs::Shape shape_;

    // test
    cob_3d_mapping_msgs::ShapeArray sa_ ;
    // end test
    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler menu_handler_for_text_;

    Eigen::Affine3f transformation_;
    Eigen::Affine3f transformation_inv_;
    int shape_ctr_ ;//ctr_ ;
//    std::vector<int> shape_indexes;

    int ctr_for_shape_indexes ;

    int display_arrow ;

};




#endif /* SHAPE_MARKER_H_ */
