/*
 * keyframe_detector.cpp
 *
 *  Created on: Sep 22, 2011
 *      Author: goa-jh
 */



//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <fstream>


// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cob_env_model/point_types.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/field_of_view_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model_msgs/GetFieldOfView.h>
#include "cob_env_model_msgs/TriggerMappingAction.h"
#include <cob_env_model_msgs/SetReferenceMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "cob_env_model/map/point_map.h"

#include "reconfigureable_node.h"
#include <cob_env_model/keyframe_detectorConfig.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model_msgs/GetFieldOfView.h>
#include "cob_env_model_msgs/TriggerMappingAction.h"
#include <cob_env_model_msgs/SetReferenceMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>


using namespace tf;

//####################
//#### node class ####
class KeyframeDetector : protected Reconfigurable_Node<cob_env_model::keyframe_detectorConfig>
{
  typedef pcl::PointXYZRGB Point;

public:
  // Constructor
  KeyframeDetector()
  : Reconfigurable_Node<cob_env_model::keyframe_detectorConfig>("KeyframeDetector"),
    first_(true)
    {
    point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &KeyframeDetector::pointCloudSubCallback, this);
    keyframe_trigger_client_ = n_.serviceClient<cob_srvs::Trigger>("trigger_keyframe");

    n_.param("aggregate_point_map/r_limit",r_limit_,0.1);
    n_.param("aggregate_point_map/y_limit",y_limit_,0.1);
    n_.param("aggregate_point_map/p_limit",p_limit_,0.1);
    n_.param("aggregate_point_map/distance_limit",distance_limit_,0.3);

    setReconfigureCallback(boost::bind(&callback, this, _1, _2));
    }


  // Destructor
  ~KeyframeDetector()
  {
    /// void
  }

  // callback for dynamic reconfigure
  static void callback(KeyframeDetector *inst, cob_env_model::keyframe_detectorConfig &config, uint32_t level)
  {
    if(!inst)
      return;

    boost::mutex::scoped_lock l2(inst->m_mutex_pointCloudSubCallback);

    inst->r_limit_ = config.r_limit;
    inst->p_limit_ = config.p_limit;
    inst->y_limit_ = config.y_limit;
    inst->distance_limit_ = config.distance_limit;

  }

  void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    boost::mutex::scoped_lock l(m_mutex_pointCloudSubCallback);

    boost::timer t;
    pcl::PointCloud<Point>::Ptr pc = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);

    StampedTransform transform;
    try
    {
      std::stringstream ss2;
      tf_listener_.waitForTransform("/map", pc_in->header.frame_id, pc_in->header.stamp, ros::Duration(0.1));
      tf_listener_.lookupTransform("/map", pc_in->header.frame_id, pc_in->header.stamp/*ros::Time(0)*/, transform);
      KDL::Frame frame_KDL, frame_KDL_old;
      tf::TransformTFToKDL(transform, frame_KDL);
      tf::TransformTFToKDL(transform_old_, frame_KDL_old);
      double r,p,y;
      frame_KDL.M.GetRPY(r,p,y);
      double r_old,p_old,y_old;
      frame_KDL_old.M.GetRPY(r_old,p_old,y_old);

      if(first_ || fabs(r-r_old) > r_limit_ || fabs(p-p_old) > p_limit_ || fabs(y-y_old) > y_limit_ ||
          transform.getOrigin().distance(transform_old_.getOrigin()) > distance_limit_)
      {
        transform_old_ = transform;
        first_=false;
        triggerKeyFrame();
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[keyframe_detector] : %s",ex.what());
    }
  }

  ros::NodeHandle n_;
  ros::Time stamp_;

  void triggerKeyFrame() {
    cob_srvs::Trigger::Request req;
    cob_srvs::Trigger::Response res;
    if(keyframe_trigger_client_.call(req,res))
    {
      ROS_DEBUG("KeyFrame service called [OK].");
    }
    else
    {
      ROS_WARN("KeyFrame service called [FAILED].");
      return;
    }
  }


protected:
  ros::Subscriber point_cloud_sub_;             //subscriber for input pc
  ros::ServiceClient keyframe_trigger_client_;
  TransformListener tf_listener_;

  bool first_;

  StampedTransform transform_old_;


  double y_limit_;
  double distance_limit_;
  double r_limit_;
  double p_limit_;

  boost::mutex m_mutex_pointCloudSubCallback;

};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "keyframe_detector");

  KeyframeDetector kd;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    //fov.transformNormals();
    loop_rate.sleep();
  }
}
