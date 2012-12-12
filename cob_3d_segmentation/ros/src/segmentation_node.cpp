
// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <cob_3d_segmentation/general_segmentation.h>
#include <cob_3d_segmentation/quad_regression/quad_regression.h>


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
class Segmentation_Node : public Parent
{
  typedef pcl::PointCloud<Point> PointCloud;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher  point_cloud_pub_;

  GeneralSegmentation<Point, PointLabel> *seg_;

  std::string algo_;
public:
  // Constructor
  Segmentation_Node()
  {
  }

  virtual ~Segmentation_Node()
  {}

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);
    point_cloud_sub_ = this->n_.subscribe("/camera/rgb/points", 1, &Segmentation_Node<Point, PointLabel, Parent>::pointCloudSubCallback, this);
    point_cloud_pub_ = n->advertise<PointCloud>("point_cloud_labeled", 1);

    //decide which algorithm should be used
    seg_ = NULL;

    if(n->getParam("algorithm",algo_))
    {
      if(algo_=="quad regression")
      {
        Segmentation::Segmentation_QuadRegression<Point,PointLabel> *seg = new Segmentation::Segmentation_QuadRegression<Point,PointLabel>();
        seg_ = seg;

        double filter;
        if(this->n_.getParam("filter",filter))
          seg->setFilter((float)filter);

        bool only_planes;
        if(this->n_.getParam("only_planes",only_planes))
          seg->setOnlyPlanes(only_planes);
      }
      else
        ROS_ERROR("%s is no valid segmentation algorithm", algo_.c_str());
    }
    else
    {
      ROS_ERROR("no valid segmentation algorithm selected");
      seg_ = new Segmentation::Segmentation_QuadRegression<Point,PointLabel>();
    }
  }

  void
  pointCloudSubCallback(const boost::shared_ptr<const PointCloud>& pc_in)
  {
    ROS_DEBUG("segmentation: point cloud callback");

    if(!seg_)
      return;

    seg_->setInputCloud(pc_in);
    seg_->compute();
    if(point_cloud_pub_.getNumSubscribers()>0)
    {
      pcl::PointCloud<PointLabel> pc_out = *seg_->getOutputCloud();
      pc_out.header = pc_in->header;
      point_cloud_pub_.publish(pc_out);
    }
  }
};

#ifdef COMPILE_NODELET

typedef Segmentation_Node<pcl::PointXYZ,pcl::PointXYZRGB,As_Nodelet> Segmentation_Node_XYZ;

PLUGINLIB_DECLARE_CLASS(cob_3d_segmentation, Segmentation_Node_XYZ, Segmentation_Node_XYZ, nodelet::Nodelet)

#else

int main(int argc, char **argv) {
  ros::init(argc, argv, "segmentation");

  Segmentation_Node<pcl::PointXYZ,pcl::PointXYZRGB,As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
