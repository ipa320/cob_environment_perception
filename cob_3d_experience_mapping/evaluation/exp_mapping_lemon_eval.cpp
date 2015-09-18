#include <cob_3d_experience_mapping/mapping.h>
#include <boost/program_options.hpp>
#include <cob_3d_mapping_common/stop_watch.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Geometry> 


#include <ratslam_ros/ViewTemplate.h>
#include <nav_msgs/Odometry.h>

/*#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <cob_3d_experience_mapping/visualization/graph.hpp>
#include "../include/ros_node.hpp"*/


#define NUM_TRANS 	2
#define NUM_ROT		1

typedef float Scalar;
typedef lemon::ListDigraph TGraph;
typedef cob_3d_experience_mapping::TransformationLink<Scalar, NUM_TRANS, NUM_ROT> TTransformationLink;
typedef cob_3d_experience_mapping::State<cob_3d_experience_mapping::Empty, Scalar, TGraph, TTransformationLink> State;
typedef cob_3d_experience_mapping::Feature<State, cob_3d_experience_mapping::Empty> Feature;
typedef cob_3d_experience_mapping::Transformation<TTransformationLink, typename State::TPtr> Transformation;
typedef cob_3d_experience_mapping::Context<Scalar /*energy*/, State /*state*/, Feature, Eigen::Matrix<float,1,2>/*energy weight*/, Transformation/*tranformation*/> TContext;
typedef TGraph::NodeMap<typename State::TPtr> TMapStates;
typedef TGraph::ArcMap <typename Transformation::TPtr> TMapTransformations;
	
struct cmpByEigenVector {
    bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
		if(a(0)!=b(0)) return a(0)<b(0);
		if(a(1)!=b(1)) return a(1)<b(1);
		return a(2)<b(2);
    }
};

int main(int argc, char **argv) {
	namespace po = boost::program_options;
	
	ros::init(argc, argv, "exp_mapping");
	
	/*****************  command line options *****************/
	std::string fn_param, fn_result, fn_out, fn_bag; //"/tmp/exp_mapping.xml.zip"
	std::string topic_feature = "";
	std::string topic_odom = "/odom";
	
	float gs_tr = 0.5f, gs_rt = 0.35f;
	int gs_num = 1000;
	
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("param", po::value<std::string>(&fn_param), "configuration file")
		("bag", po::value<std::string>(&fn_bag), "bag file")
		("result", po::value<std::string>(&fn_result), "filename to store results (loop closures, execution time)")
		("output", po::value<std::string>(&fn_out), "output path to save map")
		("topic_odom", po::value<std::string>(&topic_odom), "topic: odom")
		("topic_feature", po::value<std::string>(&topic_feature), "topic: feature")
		
		("grid_trans", po::value<float>(&gs_tr), "")
		("grid_rot", po::value<float>(&gs_rt), "")
		("grid_num", po::value<int>(&gs_num), "")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);    

	if (vm.count("help")) {
		std::cout << desc << "\n";
		return 1;
	}
	/*****************  command line options *****************/
	
	typedef std::map<Eigen::Vector3i, int, cmpByEigenVector> FeatureIdMap;
	FeatureIdMap featue_ids;
	
	TContext ctxt_;	
	TGraph graph_;
	TMapStates states_(graph_);
	TMapTransformations trans_(graph_);
	int ts=0;
	ros::Time time_last_odom_;
	
	//load settings
	{
		std::ifstream ifs(fn_param.c_str());
		if(ifs) {
			boost::archive::xml_iarchive ia(ifs);
			ia >> boost::serialization::make_nvp("settings", ctxt_.param_rw());
		}
		else {
			std::cout<<"Warning: Could not open configuration file, will create an example"<<std::endl;
			
			std::ofstream ofs(fn_param.c_str());
			boost::archive::xml_oarchive ia(ofs);
			ia << boost::serialization::make_nvp("settings", ctxt_.param());
		}
	}
	
	std::ofstream ofs_result(fn_result.c_str());
	rosbag::Bag bag;
    bag.open(fn_bag, rosbag::bagmode::Read);
	
    std::vector<std::string> topics;
    topics.push_back(std::string(topic_odom));
    topics.push_back(std::string(topic_feature));
	
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	cob_3d_experience_mapping::algorithms::init<Transformation>(graph_, ctxt_, states_, trans_);
	
	rosbag::View::iterator view_it = view.begin();
    while(view_it!=view.end()&&ros::ok())
	{
		//feature seen
        ratslam_ros::ViewTemplate::ConstPtr vid = view_it->instantiate<ratslam_ros::ViewTemplate>();
        if (vid != NULL) 
		{
			++ts;
	  
			cob_3d_experience_mapping::algorithms::reset_features(ctxt_.active_states());
			ctxt_.add_feature(vid->current_id, ts);
		}
		
		//odom
        nav_msgs::Odometry::ConstPtr odom = view_it->instantiate<nav_msgs::Odometry>();
        if (odom != NULL) 
		{
		  Eigen::Vector3f dbg_pose;
		  dbg_pose(0) = odom->pose.pose.position.x;
		  dbg_pose(1) = odom->pose.pose.position.y;
		  {
			  Eigen::Quaternionf q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z);
			  dbg_pose(2) = q.toRotationMatrix().eulerAngles(2,0,2)(2);
		  }
		  
		  if(topic_feature.size()==0)
		  {	//generate custom feature from odom
			  Eigen::Vector3i v;
			  v(0) = dbg_pose(0)/gs_tr;
			  v(1) = dbg_pose(1)/gs_tr;
			  v(2) = dbg_pose(2)/gs_rt;
			  FeatureIdMap::const_iterator it = featue_ids.find(v);
			  
			  if(it==featue_ids.end()) {
				  if(featue_ids.size()<gs_num)
					it = featue_ids.insert(std::pair<Eigen::Vector3i, int>(v, featue_ids.size())).first;
				  else
					it = featue_ids.insert(std::pair<Eigen::Vector3i, int>(v, rand()%gs_num)).first;
			  }
			  
			  ++ts;
			  cob_3d_experience_mapping::algorithms::reset_features(ctxt_.active_states());
			  ctxt_.add_feature(it->second, ts);
		  }
		  
		  if(odom->header.stamp-time_last_odom_<ros::Duration(100)) {
			  
			  typename Transformation::TLink link;
			  link(0) = odom->twist.twist.linear.x;
			  link(1) = odom->twist.twist.linear.y;
			  link(2) = odom->twist.twist.angular.z;
			  if(link(2)>M_PI)  link(2) -= 2*M_PI;
			  if(link(2)<-M_PI) link(2) += 2*M_PI;
			  link *= (odom->header.stamp-time_last_odom_).toSec();
			  
			  if(link.squaredNorm()>0) {
			  
				  Transformation action(link, ctxt_.current_active_state());
				  action.deviation() = ctxt_.param().deviation_factor_*action.dist(ctxt_.param().prox_thr_);
				  
				  PrecisionStopWatch sw;
				  cob_3d_experience_mapping::algorithms::step(graph_, ctxt_, states_, trans_, action, dbg_pose);
				  double time = time = sw.precisionStop();
				  
				  bool match_gt = true;
				  int hops = 0;
				  Eigen::Vector3f cur_pose(0,0,0);
				  if(ctxt_.active_states().size()>0) {
					  cur_pose = ctxt_.current_active_state()->dbg().pose_;
					  hops = ctxt_.current_active_state()->hops();
					match_gt &= (dbg_pose.head<2>()-cur_pose.head<2>()).norm()<ctxt_.param().prox_thr_(0);
					match_gt &= (dbg_pose.tail<1>()-cur_pose.tail<1>()).norm()<ctxt_.param().prox_thr_(1);
				  }
				  
				  ofs_result<<time<<";"<<match_gt<<";"<<(ctxt_.active_states().size()>0 ? ctxt_.current_active_state()->id()-1 : -1)
					<<";"<<dbg_pose(0)<<";"<<dbg_pose(1)<<";"<<dbg_pose(2) <<";"<<cur_pose(0)<<";"<<cur_pose(1)<<";"<<cur_pose(2)
					<<";"<<hops <<std::endl;
				}
			  
		  } else {
			  ctxt_.current_active_state()->dbg().pose_ = dbg_pose;
		  }
		  time_last_odom_ = odom->header.stamp;
		}
		
		view_it++;
	}
	
	bag.close();
	
	if(vm.count("output")>0)
		cob_3d_experience_mapping::serialization::save_content<boost::archive::binary_oarchive>(
			cob_3d_experience_mapping::ContextContainer<TContext,TGraph,TMapStates,TMapTransformations>(ctxt_, graph_, states_, trans_),
			fn_out.c_str(), true);
	
	return 0;
}
