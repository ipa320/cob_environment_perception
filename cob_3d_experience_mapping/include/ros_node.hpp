
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

	cob_3d_experience_mapping::Context<Scalar /*energy*/, State /*state*/, Eigen::Matrix<float,1,2>/*energy weight*/> ctxt_;
	
	lemon::ListDigraph graph_;
	lemon::ListDigraph::NodeMap<typename State::TPtr> cells_;
	lemon::ListDigraph::ArcMap <typename Transformation::TPtr> trans_;
	
	boost::shared_ptr<VisualizationHandler> vis_;
	
public:
  // Constructor
  ROS_Node():
	cells_(graph_),
	trans_(graph_)
  {
  }

  virtual ~ROS_Node()
  {}

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);

	ros::Subscriber sub_odometry = n->subscribe<nav_msgs::Odometry>("/odom", 0, boost::bind(&ROS_Node::on_odom, this, _1), ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());
    bool visualization_enabled;
    n->param<bool>("visualization_enabled", visualization_enabled, true);                                                             
	if(visualization_enabled)
	  vis_.reset(new VisualizationHandler);
	
	if(vis_) {
		vis_.init();
	}
  }
  
  void on_odom(const nav_msgs::Odometry::ConstPtr &odom) {
	cob_3d_experience_mapping::algorithms::step(graph_, ctxt_, cells_, trans_);
	
	if(vis_) {
		vis_.visualize(graph_, cells_, typename State::TPtr());
	}
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
