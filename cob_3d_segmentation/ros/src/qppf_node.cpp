/*
 * qppf_node.cpp
 *
 *  Created on: 21.07.2012
 *      Author: josh
 */



// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <cob_3d_segmentation/quad_regression/quad_regression.h>
#include <cob_3d_mapping_msgs/CurvedPolygon_Array.h>

class As_Node
{
protected:
  ros::NodeHandle n_;
public:
  As_Node() {
  }

  virtual ~As_Node() {}

  virtual void onInit()=0;

  void start() {

  }
};

class As_Nodelet : public  pcl_ros::PCLNodelet
{
protected:
  ros::NodeHandle n_;
public:
  As_Nodelet() {
  }

  virtual ~As_Nodelet() {}

  void start() {
    PCLNodelet::onInit();
    n_ = getNodeHandle();
  }
};

template <typename Point, typename PointLabel, typename Parent>
class QPPF_Node : public Parent
{
  typedef pcl::PointCloud<Point> PointCloud;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher  curved_pub_, shapes_pub_;

  Segmentation::Segmentation_QuadRegression<Point, PointLabel> seg_;

public:
  // Constructor
  QPPF_Node()
  {
  }

  virtual ~QPPF_Node()
  {}

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);
    point_cloud_sub_ = this->n_.subscribe("/camera/depth/points", 1, &QPPF_Node<Point, PointLabel, Parent>::pointCloudSubCallback, this);
    curved_pub_ = n->advertise<cob_3d_mapping_msgs::CurvedPolygon_Array>("/curved_polygons", 1);
    shapes_pub_ = n->advertise<cob_3d_mapping_msgs::ShapeArray>("/shapes_array", 1);

  }

  void
  pointCloudSubCallback(const boost::shared_ptr<const PointCloud>& pc_in)
  {
    ROS_DEBUG("segmentation: point cloud callback");

    seg_.setInputCloud(pc_in);
    seg_.compute();
    if(shapes_pub_.getNumSubscribers()>0)
    {
      cob_3d_mapping_msgs::ShapeArray sa = seg_;
      sa.header.stamp = pc_in->header.stamp;
      shapes_pub_.publish(sa);
    }
    if(curved_pub_.getNumSubscribers()>0)
    {
      cob_3d_mapping_msgs::CurvedPolygon_Array cpa;
      for(size_t i=0; i<seg_.getPolygons().size(); i++)
      {
        cob_3d_mapping_msgs::CurvedPolygon cp;
        seg_.getPolygons()[i].toRosMsg(&cp,pc_in->header.stamp);
        cpa.polygons.push_back(cp);
      }
      cpa.header = pc_in->header;
      curved_pub_.publish(cpa);
    }
  }
};

#ifdef COMPILE_NODELET

typedef QPPF_Node<pcl::PointXYZ,pcl::PointXYZRGB,As_Nodelet> QPPF_XYZ;

PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, QPPF_XYZ, QPPF_Node_XYZ, nodelet::Nodelet)

#else

int main(int argc, char **argv) {
  ros::init(argc, argv, "qppf");

  QPPF_Node<pcl::PointXYZ,pcl::PointXYZRGB,As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
