
#include <nav_msgs/Odometry.h>

class As_Node
{
protected:
  ros::NodeHandle n_;
public:
  As_Node(): n_("~") {
  }

  virtual ~As_Node() {}

  virtual void onInit()=0;

  void start() {

  }
};

#ifdef COMPILE_NODELET

#include <nodelet.h>

class As_Nodelet : public  nodelet::Nodelet
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
#endif

template <typename Parent, int NUM_TRANS=2, int NUM_ROT=1, typename Scalar=float>
class ROS_Node : public Parent
{
public:
	typedef cob_3d_experience_mapping::State<cob_3d_experience_mapping::Empty, Scalar > State;
	typedef cob_3d_experience_mapping::Transformation<Scalar, NUM_TRANS, NUM_ROT, typename State::TPtr> Transformation;
	typedef cob_3d_experience_mapping::visualization::VisualizationHandler<typename State::TGraph, lemon::ListDigraph::NodeMap<typename State::TPtr>, typename State::TPtr, typename State::TArcIterator> VisualizationHandler;
	
private:

	cob_3d_experience_mapping::Context<Scalar /*energy*/, State /*state*/, Eigen::Matrix<float,1,2>/*energy weight*/, Transformation/*tranformation*/> ctxt_;
	
	lemon::ListDigraph graph_;
	lemon::ListDigraph::NodeMap<typename State::TPtr> cells_;
	lemon::ListDigraph::ArcMap <typename Transformation::TPtr> trans_;
	
	boost::shared_ptr<VisualizationHandler> vis_;
	
	ros::Subscriber sub_odometry_;
	ros::Time time_last_odom_;
public:
  // Constructor
  ROS_Node():
	cells_(graph_),
	trans_(graph_)
  {
  }

  virtual ~ROS_Node()
  {}

  //TODO: use dyn. reconfig.
  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);

	sub_odometry_ = n->subscribe<nav_msgs::Odometry>("/odom", 0, boost::bind(&ROS_Node::on_odom, this, _1), ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());
    bool visualization_enabled;
    n->param<bool>("visualization_enabled", visualization_enabled, true);                                                             
	if(visualization_enabled)
	  vis_.reset(new VisualizationHandler);
	
	/*if(vis_) {
		vis_->init();
	}*/

	  cob_3d_experience_mapping::algorithms::init<Transformation>(graph_, ctxt_, cells_, trans_);
  }
  
  void on_odom(const nav_msgs::Odometry::ConstPtr &odom) {
	  ROS_INFO("-------------------------------");
	  if(time_last_odom_.isValid() && (odom->header.stamp-time_last_odom_)<ros::Duration(10)) {
		  ROS_INFO("on odom.");

		  typename Transformation::TLink link;
		  link(0) = odom->twist.twist.linear.x;
		  link(1) = odom->twist.twist.linear.y;
		  link(2) = odom->twist.twist.angular.z;
		  link *= (odom->header.stamp-time_last_odom_).toSec();

		  ROS_INFO("odom: %f %f %f", link(0),link(1),link(2));

		  Transformation action(link, ctxt_.current_active_cell());
		  cob_3d_experience_mapping::algorithms::step(graph_, ctxt_, cells_, trans_, action);

		  if(vis_ && ctxt_.active_cells().size()>0) {
			vis_->visualize(graph_, cells_, ctxt_.current_active_cell());
		  }
	  } else
		  ROS_INFO("skipped odom");
	  time_last_odom_ = odom->header.stamp;
  }
};

/*
#ifdef COMPILE_NODELET

typedef ROS_Node<pcl::PointXYZ,pcl::PointXYZRGB,As_Nodelet> _ROS_Nodelet;

PLUGINLIB_DECLARE_CLASS(cob_3d_mapping_slam, _ROS_Nodelet, _ROS_Nodelet, nodelet::Nodelet)

#else

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam");

  ROS_Node<As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
*/
