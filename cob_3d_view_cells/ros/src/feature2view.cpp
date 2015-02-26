/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: cob_3d_view_cells
 *
 * \author
 *  Author: Joshua Hampp
 *
 * \date Date of creation: 31.10.2014
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>
#include <pcl/io/ply_io.h>
    
#include <cob_3d_mapping_common/node_skeleton.h>

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <std_msgs/Int32.h>

#include <cob_3d_mapping_common/point_types.h>

template<typename ID>
ID create_new_id() {
	static ID nid = 0;
	return nid++;
}

#include <cob_3d_view_cells/input.h>
#include <cob_3d_view_cells/space.h>
#include <cob_3d_view_cells/matcher.h>

template <typename FeaturePoint, typename Parent>
class Feature2View_Node : public Parent
{
  typedef pcl::PointXYZ SimplePoint;
  typedef pcl::PointCloud<SimplePoint> SimplePointCloud;
  typedef pcl::PointCloud<FeaturePoint> PointCloud;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> PCSyncPolicy;

  message_filters::Subscriber<sensor_msgs::PointCloud2> 			point_cloud_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> 	keypoints_sub_;
  ros::Publisher  view_pub_;
  boost::shared_ptr<message_filters::Synchronizer<PCSyncPolicy> > sync_pcs_;
  
  typedef SearchSpace<FeaturePoint> TSearchSpace;
  TSearchSpace search_;
  
  double int_thr_;

public:
  // Constructor
  Feature2View_Node()
  {
  }

  virtual ~Feature2View_Node()
  {}

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);
    point_cloud_sub_.subscribe(this->n_, "/features", 1/*, &Feature2View_Node<FeaturePoint, Parent>::pointCloudSubCallback, this*/);
    keypoints_sub_.subscribe(this->n_, "/keypoints", 1/*, &Feature2View_Node<FeaturePoint, Parent>::pointCloudSubCallback, this*/);
    view_pub_ = n->advertise<std_msgs::Int32>("view_id", 1);
    
    sync_pcs_.reset(new message_filters::Synchronizer<PCSyncPolicy>(PCSyncPolicy(2), point_cloud_sub_, keypoints_sub_));
    sync_pcs_->registerCallback( boost::bind( &Feature2View_Node<FeaturePoint, Parent>::pointCloudSubCallback, this, _1, _2 ) );
    
    double dist_thr;
    n->param<double>("dist_thr", dist_thr, 90.);
    search_.setDistThreshold(dist_thr);
    
    n->param<double>("int_thr", int_thr_, 0.4);
    
    g_dbg_reg = new DebugRegistration(n);

    /*double filter;
    if(this->n_.getParam("filter",filter))
      seg_.setFilter((float)filter);*/
      
    ROS_INFO("using params: %f %f", dist_thr, int_thr_);
  }
  
  /**
   * function for evaluation (to speed up)
   */
  void readFromBag(rosbag::Bag &bag, rosbag::Bag &bag_out) {
    rosbag::View view(bag);
    tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;

	sensor_msgs::PointCloud2ConstPtr last_pc, last_kp;
	std::map<int, Eigen::Vector3f> id2pos;
	std::map<int, Eigen::Quaternionf> id2rot;
	std::vector<Eigen::Vector3f> list_pos;
	std::vector<Eigen::Quaternionf> list_rot;
	std::map<int, sensor_msgs::PointCloud2> id2pc;
	std::map<int, double> id2ts;
	
	int stats[7]={};
	enum {
		STAT_FRAMES=0, STAT_FRAMES_WITH_TF=1, STAT_MATCH_SUCCESS=2, STAT_MATCH_FAILURE=3,
		STAT_NO_MATCH=4, STAT_GS_MATCH=5, STAT_GS_NOMATCH=6};
	
	const float thr_dist = 0.6f, thr_rot = 0.1f;
	double start=-1;
	
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
		bag_out.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
		
		// Handle TF messages first
		tf::tfMessage::ConstPtr tf = m.instantiate<tf::tfMessage> ();
		if (tf != NULL)
		{
			tf_broadcaster.sendTransform (tf->transforms);
			ros::spinOnce ();
			usleep(1000*33);
		}
		
        sensor_msgs::PointCloud2ConstPtr pc  = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc != NULL) {
			//ROS_INFO("stamp %f %s", pc->header.stamp.toSec(), m.getTopic().c_str());
			if(m.getTopic()=="/keypoints") last_kp = pc;
			else if(m.getTopic()=="/features") last_pc = pc;
		}
		
		if(last_pc && last_kp && std::abs(last_pc->header.stamp.toSec()-last_kp->header.stamp.toSec())<0.002f) {
			if(start<0) start = last_kp->header.stamp.toSec();
			
			 std_msgs::Int32 msg;
			 if(_pointCloudSubCallback(last_pc,last_kp, msg)) {
				bag_out.write("/feature2view_eval/view_id",last_pc->header.stamp,msg);
				stats[STAT_FRAMES]++;
			 
				 tf::StampedTransform transform;
				 try {
					tf_listener.lookupTransform("/openni_camera", "/world", ros::Time(0), transform);
					
					if(std::abs(last_pc->header.stamp.toSec()-last_kp->header.stamp.toSec())<0.2f) {
						stats[STAT_FRAMES_WITH_TF]++;
						Eigen::Vector3f v(
							transform.getOrigin().x(),
							transform.getOrigin().y(),
							transform.getOrigin().z());
						Eigen::Quaternionf r(
							transform.getRotation().w(),
							transform.getRotation().x(),
							transform.getRotation().y(),
							transform.getRotation().z());
							
						if(id2pos.find(msg.data)!=id2pos.end()) std::cout<<"DDD "<<(id2pos[msg.data]-v).norm()<<" "<<((id2rot[msg.data].toRotationMatrix()*r.toRotationMatrix().transpose()).diagonal()-Eigen::Vector3f(1,1,1)).norm()<<std::endl;
						
						std::cout<<"TS: "<<last_kp->header.stamp.toSec()-start<<" "<<id2ts[msg.data]<<std::endl;
						std::cout<<"RESULT: ";
						if(id2pos.find(msg.data)!=id2pos.end()) {
							float dC = (id2pos[msg.data]-v).norm();
							float dR = ((id2rot[msg.data].toRotationMatrix()*r.toRotationMatrix().transpose()).diagonal()-Eigen::Vector3f(1,1,1)).norm();//std::acos((r.toRotationMatrix()*Eigen::Vector3f::UnitZ()).dot(id2rot[msg.data].toRotationMatrix()*Eigen::Vector3f::UnitZ()));
							std::cout<<dC<<" "<<dR<<" "<<(dC<=thr_dist || dR<thr_rot);
							stats[(dC<=thr_dist || dR<thr_rot)?STAT_MATCH_SUCCESS:STAT_MATCH_FAILURE]++;
							
							char fn[512];
							{SimplePointCloud pc;
							pcl::fromROSMsg(id2pc[msg.data],pc);
							sprintf(fn,"/tmp/match%d_ref.pcd",msg.data);
							pcl::io::savePCDFile(fn, pc);}
							{SimplePointCloud pc;
							pcl::fromROSMsg(*last_kp,pc);
							sprintf(fn,"/tmp/match%d.pcd",msg.data);
							pcl::io::savePCDFile(fn, pc);}
						} else {
							id2pos[msg.data] = v;
							id2rot[msg.data] = r;
							id2ts[msg.data] = last_kp->header.stamp.toSec()-start;
							id2pc[msg.data] = *last_kp;
							std::cout<<"-1 -1 "<<false;
							stats[STAT_NO_MATCH]++;
						}
						
						bool found=false;
						size_t mm=0;
						for(size_t i=0; !found && i+10<list_pos.size(); i++) {
							float dC = (list_pos[i]-v).norm();
							float dR = ((list_rot[i].toRotationMatrix()*r.toRotationMatrix().transpose()).diagonal()-Eigen::Vector3f(1,1,1)).norm();//std::acos((r.toRotationMatrix()*Eigen::Vector3f::UnitZ()).dot(list_rot[i].toRotationMatrix()*Eigen::Vector3f::UnitZ()));
							//std::cout<<dC<<" "<<dR<<std::endl;
							found = (dC<=thr_dist && dR<thr_rot);
							mm=i;
						}
						std::cout<<" "<<found<<std::endl;
						stats[found?STAT_GS_MATCH:STAT_GS_NOMATCH]++;
						
						std::cout<<"MATCH "<<mm<<" "<<list_pos.size()<<std::endl;
						list_pos.push_back(v);
						list_rot.push_back(r);
						
						if(found) std::cout<<"FFF "<<(list_pos[mm]-v).norm()<<" "<<((list_rot[mm].toRotationMatrix()*r.toRotationMatrix().transpose()).diagonal()-Eigen::Vector3f(1,1,1)).norm()<<std::endl;
					} else {
						std::cout<<"ERROR: timestamps to far appart"<<std::endl;
					}
				} 
				catch (tf::TransformException ex){
				  ROS_INFO("%s",ex.what());
				}
			}
			
			 last_pc.reset();
			 last_kp.reset();
		}
    }
    
    std::cout<<"STAT:";
    for(int i=0; i<sizeof(stats)/sizeof(stats[0]); i++)
		std::cout<<" "<<stats[i];
	std::cout<<std::endl;
  }

  bool
  _pointCloudSubCallback(const sensor_msgs::PointCloud2ConstPtr& pc_in2, const sensor_msgs::PointCloud2ConstPtr& kps_pc_in2, std_msgs::Int32 &msg)
  {
    ROS_DEBUG("view cells: point cloud callback");
    PointCloud pc_in;
    SimplePointCloud::Ptr kps_pc_in(new SimplePointCloud);
	pcl::fromROSMsg(*pc_in2, pc_in);
	pcl::fromROSMsg(*kps_pc_in2, *kps_pc_in);
    
    if(pc_in.size()<1) return false;
		
	Matcher<typename TSearchSpace::TContent, SimplePoint> matcher;
	matcher.setIntersectionThreshold(int_thr_);
	matcher.setKeypoints(kps_pc_in);
	
	std::vector<typename TSearchSpace::ContentPtr> cnts;
	for(size_t i=0; i<pc_in.size(); i++) {
		std::vector<typename TSearchSpace::ContentPtr> tmp = search_.lookup(pc_in[i]);
		cnts.insert(cnts.end(), tmp.begin(),tmp.end());
		matcher.push_back(tmp, pc_in[i]);
	}
	search_.finish();
	
	msg.data = matcher.get_id();
	
	//for(size_t i=0; i<cnts.size(); i++)
	//	ROS_INFO("c %d", (int)cnts[i]->size());
	
	ROS_INFO("--------------------------");
	//getchar();
	return true;
  }
	
  void
  pointCloudSubCallback(const sensor_msgs::PointCloud2ConstPtr& pc_in2, const sensor_msgs::PointCloud2ConstPtr& kps_pc_in2) {
	std_msgs::Int32 msg;
	if(_pointCloudSubCallback(pc_in2, kps_pc_in2, msg)) {
		const bool subscribers =
			(view_pub_.getNumSubscribers()>0);
		
		if(!subscribers) {
			ROS_DEBUG("view cells: no subscribers --> do nothing");
			return;
		}
	
		view_pub_.publish(msg);
	}
  }
};

#ifdef COMPILE_NODELET

typedef Feature2View_Node<pcl::PointXYZRGB,As_Nodelet> Feature2View_Nodelet_XYZ;

PLUGINLIB_DECLARE_CLASS(cob_3d_view_cells, Feature2View_Nodelet_XYZ, nodelet::Nodelet)

#else

#include <boost/program_options.hpp>
int main(int argc, char **argv) {
  ros::init(argc, argv, "feature2view_eval");

  Feature2View_Node<PointXYZFeature64,As_Node> sn;
  sn.onInit();
  
  namespace po = boost::program_options;
  
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options() ("bag", po::value<std::string>(), "set bag file to replay");
  desc.add_options() ("bag_out", po::value<std::string>(), "set bag file to write");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    

  if(vm.count("bag") && vm.count("bag_out")) {
	rosbag::Bag bag, bag_out;
    bag.open(vm["bag"].as<std::string>(), rosbag::bagmode::Read);
    bag_out.open(vm["bag_out"].as<std::string>(), rosbag::bagmode::Write);
    bag_out.setCompression(rosbag::compression::BZ2);
    sn.readFromBag(bag, bag_out);
    bag.close();
    bag_out.close();
  }
  else ros::spin();

  return 0;
}

#endif
