//TODO: header, adjust subscriber and other variable names

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


// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>

// ROS message includes
#include <sensor_msgs/CameraInfo.h>
#include <cob_srvs/Trigger.h>


#include "cob_3d_mapping_common/reconfigureable_node.h"
#include <cob_3d_mapping_common/keyframe_detectorConfig.h>



using namespace tf;
using namespace cob_3d_mapping_common;

//####################
//#### node class ####
class KeyframeDetector : protected Reconfigurable_Node<keyframe_detectorConfig>
{

public:
  // Constructor
  KeyframeDetector()
  : Reconfigurable_Node<keyframe_detectorConfig>("KeyframeDetector"),
    first_(true), trigger_always_(false)
    {
    point_cloud_sub_ = n_.subscribe("camera_info", 1, &KeyframeDetector::pointCloudSubCallback, this);
    transform_sub_ = n_.subscribe("/tf", 1, &KeyframeDetector::transformSubCallback, this);
    keyframe_trigger_client_ = n_.serviceClient<cob_srvs::Trigger>("trigger_keyframe");

    n_.param("aggregate_point_map/r_limit",r_limit_,0.1);
    n_.param("aggregate_point_map/y_limit",y_limit_,0.1);
    n_.param("aggregate_point_map/p_limit",p_limit_,0.1);
    n_.param("aggregate_point_map/distance_limit",distance_limit_,0.3);
    n_.param("aggregate_point_map/trigger_always",trigger_always_,false);

    setReconfigureCallback(boost::bind(&callback, this, _1, _2));
    }


  // Destructor
  ~KeyframeDetector()
  {
    /// void
  }

  // callback for dynamic reconfigure
  static void callback(KeyframeDetector *inst, keyframe_detectorConfig &config, uint32_t level)
  {
    if(!inst)
      return;

    boost::mutex::scoped_lock l2(inst->m_mutex_pointCloudSubCallback);

    inst->r_limit_ = config.r_limit;
    inst->p_limit_ = config.p_limit;
    inst->y_limit_ = config.y_limit;
    inst->distance_limit_ = config.distance_limit;
    inst->trigger_always_ = config.trigger_always;

  }

  //TODO: better listen to camera_info
  void
  pointCloudSubCallback(sensor_msgs::CameraInfo::ConstPtr pc_in)
  {
    frame_id_ = pc_in->header.frame_id;
    //point_cloud_sub_.shutdown();
  }

  void
  transformSubCallback(const tf::tfMessageConstPtr& msg)
  {
    boost::mutex::scoped_lock l(m_mutex_pointCloudSubCallback);

    if(frame_id_.size()<1) {
      ROS_WARN("frame id is missing");
	frame_id_="head_cam3d_link";
      //return;
    }

    //pcl::PointCloud<Point>::Ptr pc = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);

    StampedTransform transform;
    /*
          std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

          if (mapped_tgt == mapped_src) {
                  transform.setIdentity();
                  transform.child_frame_id_ = mapped_src;
                  transform.frame_id_       = mapped_tgt;
                  transform.stamp_          = now();
                  return;
          }
          */
    try
    {
      //ROS_INFO("%s", frame_id_.c_str());
      std::stringstream ss2;
      tf_listener_.waitForTransform("/map", frame_id_, ros::Time(0), ros::Duration(0.1));
      tf_listener_.lookupTransform("/map", frame_id_, ros::Time(0), transform);
      KDL::Frame frame_KDL, frame_KDL_old;
      tf::TransformTFToKDL(transform, frame_KDL);
      tf::TransformTFToKDL(transform_old_, frame_KDL_old);
      double r,p,y;
      frame_KDL.M.GetRPY(r,p,y);
      double r_old,p_old,y_old;
      frame_KDL_old.M.GetRPY(r_old,p_old,y_old);

      if(trigger_always_ || first_ || fabs(r-r_old) > r_limit_ || fabs(p-p_old) > p_limit_ || fabs(y-y_old) > y_limit_ ||
          transform.getOrigin().distance(transform_old_.getOrigin()) > distance_limit_)
      {
        if(triggerKeyFrame()) {
          transform_old_ = transform;
          first_=false;
        }
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[keyframe_detector] : %s",ex.what());
    }
  }

  ros::NodeHandle n_;
  ros::Time stamp_;

  bool triggerKeyFrame() {
    cob_srvs::Trigger::Request req;
    cob_srvs::Trigger::Response res;
    if(keyframe_trigger_client_.call(req,res))
    {
      ROS_DEBUG("KeyFrame service called [OK].");
    }
    else
    {
      ROS_WARN("KeyFrame service called [FAILED].", res.success.data);
      return false;
    }

    return res.success.data;
  }


protected:
  ros::Subscriber point_cloud_sub_, transform_sub_;             //subscriber for input pc
  TransformListener tf_listener_;
  ros::ServiceClient keyframe_trigger_client_;

  bool first_;
  bool trigger_always_;

  StampedTransform transform_old_;

  std::string frame_id_;


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
