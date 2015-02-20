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
#include <boost/foreach.hpp>
    
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

	sensor_msgs::PointCloud2ConstPtr last_pc, last_kp;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
		bag_out.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
		
        sensor_msgs::PointCloud2ConstPtr pc  = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc != NULL) {
			//ROS_INFO("stamp %f %s", pc->header.stamp.toSec(), m.getTopic().c_str());
			if(m.getTopic()=="/keypoints") last_kp = pc;
			else if(m.getTopic()=="/features") last_pc = pc;
		}
		
		if(last_pc && last_kp && std::abs(last_pc->header.stamp.toSec()-last_kp->header.stamp.toSec())<0.002f) {
			 std_msgs::Int32 msg;
			 if(_pointCloudSubCallback(last_pc,last_kp, msg))
				bag_out.write("/feature2view_eval/view_id",last_pc->header.stamp,msg);
			 last_pc.reset();
			 last_kp.reset();
		}
    }
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
