
#include <nav_msgs/Odometry.h>
#include <cob_3d_experience_mapping/SensorInfoArray.h>
//#include <tf/transform_listener.h>
#include <ratslam_ros/TopologicalAction.h>
#include <std_msgs/Int32.h>
#include <Eigen/Geometry> 

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
	typedef lemon::ListDigraph TGraph;
	typedef cob_3d_experience_mapping::TransformationLink<Scalar, NUM_TRANS, NUM_ROT> TTransformationLink;
	typedef cob_3d_experience_mapping::State<cob_3d_experience_mapping::Empty, Scalar, TGraph, TTransformationLink> State;
	typedef cob_3d_experience_mapping::Feature<State, cob_3d_experience_mapping::Empty> Feature;
	typedef cob_3d_experience_mapping::Transformation<TTransformationLink, typename State::TPtr> Transformation;
#ifdef VIS_
	typedef cob_3d_experience_mapping::visualization::VisualizationHandler<typename State::TGraph, typename TGraph::NodeMap<typename State::TPtr>, typename TGraph::ArcMap <typename Transformation::TPtr>, typename State::TPtr, typename State::TArcOutIterator> VisualizationHandler;
#endif
	typedef cob_3d_experience_mapping::Context<Scalar /*energy*/, State /*state*/, Feature, Eigen::Matrix<float,1,2>/*energy weight*/, Transformation/*tranformation*/> TContext;
	typedef TGraph::NodeMap<typename State::TPtr> TMapCells;
	typedef TGraph::ArcMap <typename Transformation::TPtr> TMapTransformations;
	
private:
	
	TContext ctxt_;	
	TGraph graph_;
	TMapCells cells_;
	TMapTransformations trans_;
	boost::mutex mtx_;
	
#ifdef VIS_
	boost::shared_ptr<VisualizationHandler> vis_;
#endif

	ros::Subscriber sub_odometry_, sub_sensor_info_, sub_view_template_, sub_view_id_;
	ros::Publisher pub_top_action_;
	ros::Time time_last_odom_;
	
	bool step_mode_;
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
	sub_sensor_info_ = n->subscribe<cob_3d_experience_mapping::SensorInfoArray>("/sim_barks/sensor_info", 0, boost::bind(&ROS_Node::on_sensor_info, this, _1), ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());
                           
	sub_view_id_ = n->subscribe<std_msgs::Int32>("/feature2view_eval/view_id", 0, boost::bind(&ROS_Node::on_view_id, this, _1), ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());
                                                                    
	pub_top_action_ = n->advertise<ratslam_ros::TopologicalAction>("/PoseCell/TopologicalAction", 10);
	
#ifdef VIS_
    bool visualization_enabled;
    n->param<bool>("visualization_enabled", visualization_enabled, true);                                                             
	if(visualization_enabled)
	  vis_.reset(new VisualizationHandler);
	
	/*if(vis_) {
		vis_->init();
	}*/
#endif

    n->param<bool>("step_mode", step_mode_, false);    

	  cob_3d_experience_mapping::algorithms::init<Transformation>(graph_, ctxt_, cells_, trans_);
	  
	  printf("init done\n");
  }
  
  void on_sensor_info(const cob_3d_experience_mapping::SensorInfoArray::ConstPtr &infos) {
	  boost::lock_guard<boost::mutex> guard(mtx_);
	  
	  static int ts=0;
	  ++ts;
	  
	   cob_3d_experience_mapping::algorithms::reset_features(ctxt_.active_cells());
	  
	  for(size_t i=0; i<infos->infos.size(); i++) {
		  ctxt_.add_feature(infos->infos[i].id, ts);
	  }
  }
  
  void on_view_id(const std_msgs::Int32::ConstPtr &vid) {
	  boost::lock_guard<boost::mutex> guard(mtx_);
	  
	  static int ts=0;
	  ++ts;
	  
	   cob_3d_experience_mapping::algorithms::reset_features(ctxt_.active_cells());
	   ctxt_.add_feature(vid->data, ts);
  }
  
  void on_odom(const nav_msgs::Odometry::ConstPtr &odom) {
	  boost::lock_guard<boost::mutex> guard(mtx_);
	  
	  Eigen::Vector3f dbg_pose;
	  dbg_pose(0) = odom->pose.pose.position.x;
	  dbg_pose(1) = odom->pose.pose.position.y;
	  if(step_mode_) 
		dbg_pose(2) = odom->pose.pose.orientation.w;
	  else {
		  Eigen::Quaternionf q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z);
		  dbg_pose(2) = q.toRotationMatrix().eulerAngles(2,0,2)(2);
	  }
	  
	  ROS_INFO("-------------------------------");
	  if(/*time_last_odom_.isValid() &&*/ (odom->header.stamp-time_last_odom_)<ros::Duration(100)) {
		  ROS_INFO("on odom.");

		  typename Transformation::TLink link;
		  link(0) = odom->twist.twist.linear.x;
		  link(1) = odom->twist.twist.linear.y;
		  link(2) = odom->twist.twist.angular.z;
		  if(link(2)>M_PI)  link(2) -= 2*M_PI;
		  if(link(2)<-M_PI) link(2) += 2*M_PI;
		  if(!step_mode_) link *= (odom->header.stamp-time_last_odom_).toSec();
		  
		  else if(std::abs(link(2))>0.4) link(2) *= 0.4/std::abs(link(2)); //TODO: TESTING ONLY!!!
		  
		  if(link.squaredNorm()<=0) return;

		  ROS_INFO("debug pose: %f %f %f", dbg_pose(0),dbg_pose(1),dbg_pose(2));
		  ROS_INFO("odom: %f %f %f", link(0),link(1),link(2));

		  Transformation action(link, ctxt_.current_active_cell());
		  if(step_mode_)
			action.deviation() = 0.075f;
		  else
			action.deviation() = 0.025f*action.dist(ctxt_.param().prox_thr_);
		  
		  //cob_3d_experience_mapping::algorithms::step(graph_, ctxt_, cells_, trans_, action, dbg_pose);
		  
		  ratslam_ros::TopologicalAction rs_action;
		  rs_action.header = odom->header;
		  rs_action.action = ratslam_ros::TopologicalAction::CREATE_NODE;
		  rs_action.src_id = ctxt_.active_cells().size()>0 ? ctxt_.current_active_cell()->id()-1 : -1;
		  
		  cob_3d_experience_mapping::algorithms::step(graph_, ctxt_, cells_, trans_, action, dbg_pose);
		  
		  static int biggest=0;
		  static std::map<int,int> rs_ids;
		  if(biggest==0) {
			rs_action.dest_id = 0;
			pub_top_action_.publish(rs_action);
		  }
		  rs_action.dest_id = ctxt_.active_cells().size()>0 ? ctxt_.current_active_cell()->id()-1 : -1;
		  
		  if(rs_ids.find(rs_action.src_id)==rs_ids.end())
			rs_ids[rs_action.src_id] = rs_ids.size()-1;
		  if(rs_ids.find(rs_action.dest_id)==rs_ids.end())
			rs_ids[rs_action.dest_id] = rs_ids.size()-1;
		  rs_action.dest_id = rs_ids[rs_action.dest_id];
		  rs_action.src_id = rs_ids[rs_action.src_id];
			
		  if(rs_action.dest_id>biggest) {
			pub_top_action_.publish(rs_action);
			biggest = rs_action.dest_id;
		  }
		  else if(rs_action.src_id!=-1 && rs_action.src_id!=rs_action.dest_id) {
			rs_action.action = ratslam_ros::TopologicalAction::CREATE_EDGE;
			pub_top_action_.publish(rs_action);
		  }
		  else {
			rs_action.action = ratslam_ros::TopologicalAction::SET_NODE;
			pub_top_action_.publish(rs_action);
		  }
		  

#ifdef VIS_
		  if(vis_ && ctxt_.active_cells().size()>0) {
			vis_->visualize(graph_, cells_, trans_, ctxt_.current_active_cell());
		  }
#endif
	  } else {
		  ctxt_.current_active_cell()->dbg().pose_ = dbg_pose;
		  ROS_INFO("skipped odom %f %d", (odom->header.stamp-time_last_odom_).toSec(), (int)time_last_odom_.isValid());
	  }
	  time_last_odom_ = odom->header.stamp;
	  printf("\n");
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
