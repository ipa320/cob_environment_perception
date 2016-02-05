#pragma once

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <cob_3d_mapping_geometry_map_v2/types/context.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>

#include <cob_object_detection_msgs/DetectionArray.h>

#if ROS_VERSION_MINIMUM(1, 11, 0)
#include <std_srvs/Trigger.h>
#else
#include <cob_srvs/Trigger.h>
#endif

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


struct SCarton {
	int id_;
	Eigen::Vector3d pos_, dim_;
	int box_count_horizontal_;
	
	SCarton(XmlRpc::XmlRpcValue &item) {
		id_ = item["volume_index"];
		pos_(0) = item["x"];
		pos_(1) = (double)item["y"]-((double)item["H"]-(double)item["h"]);
		pos_(2) = item["z"];
		dim_(0) = item["w"];
		dim_(1) = item["H"];	//we use shelf height instead of height of volume
		dim_(2) = item["d"];
		box_count_horizontal_ = std::max(1, (int)item["box_count_horizontal"]);
	}
};

class GeometryNode : public cob_3d_geometry_map::TransformationEstimator {
	ros::NodeHandle nh_;
	cob_3d_geometry_map::GlobalContext::Ptr ctxt_;
	cob_3d_geometry_map::DefaultClassifier::Classifier_Floor *classifier_floor_;
	std::vector<cob_3d_geometry_map::CustomClassifier::Classifier_Carton*> classifier_cartons_;
	
	std::string target_frame_;
	Eigen::Affine3d tf2target_;

	void callback(const cob_3d_mapping_msgs::PlaneSceneConstPtr& scene, const sensor_msgs::ImageConstPtr& color_img)
	{
	  ROS_INFO("callback");
	  
	}

	void callback2(const cob_3d_mapping_msgs::PlaneSceneConstPtr& scene)
	{
	  ROS_INFO("callback2");
	  
	  if(!ctxt_) return;
	  
	  //if not set we keep in input frame
	  if(target_frame_.empty())
		target_frame_ = scene->header.frame_id;
	  
	  //update visualization frame information
	  cob_3d_visualization::RvizMarkerManager::get().setFrameId(target_frame_);
	  
	  //lookup transformation to for our geometry map
	  tf::StampedTransform transform;
	  try{
		  tf_listener_.waitForTransform(target_frame_, scene->header.frame_id, (ros::Time::now()-scene->header.stamp)>ros::Duration(5*60) ? ros::Time(0) : scene->header.stamp, ros::Duration(10));
		  tf_listener_.lookupTransform(target_frame_, scene->header.frame_id,
								   (ros::Time::now()-scene->header.stamp)>ros::Duration(5*60) ? ros::Time(0) : scene->header.stamp,	//assume we're replaying a bag file
								   transform);
	  }
	  catch (tf::TransformException ex){
		  ROS_ERROR("%s --> SKIPPING!",ex.what());
		  return;
	  }

	  tf::transformTFToEigen(transform, tf2target_);
	  
	  try {
		  //now do the mapping stuff
		  ros::Time start = ros::Time::now();
		  ctxt_->add_scene(*scene, this);
		  ROS_INFO("took %f", (ros::Time::now()-start).toSec());
		  
		  //visualize it
		  ctxt_->visualize_markers();
		  
		  //some additional features
		  publish_scan(scene->header);
		  publish_cartons(scene->header);
	  }
	  catch(...) {
		  ROS_ERROR("some error in geometry_map_v2, resetting...");
		  reset();
	  }
	}
	
	void publish_cartons(const std_msgs::Header &header) {
		if(pub_cartons_.getNumSubscribers()<1) return;
		cob_object_detection_msgs::DetectionArray array;
		array.header = header;
		array.header.frame_id = target_frame_;
		
		for(size_t i=0; i<classifier_cartons_.size(); i++) {
			std::vector<cob_3d_geometry_map::ObjectVolume> cartons = classifier_cartons_[i]->get_cartons();
			for(size_t j=0; j<cartons.size(); j++) {
				cob_object_detection_msgs::Detection msg;
				msg.header = array.header;
				msg.detector = "geometry_map::Classsifier::carton";
				
				msg.label = classifier_cartons_[i]->name();
				msg.id = classifier_cartons_[i]->carton_id();		//ids with leading n*100 is a virtual box, so correct id to match the planogram
				
				msg.pose.pose.position.x = cartons[j].pose().loc_.X();
				msg.pose.pose.position.y = cartons[j].pose().loc_.Y();//+cartons[j].bb_in_pose().sizes()(1);
				msg.pose.pose.position.z = cartons[j].pose().loc_.Z();
				
				msg.pose.pose.orientation.x = cartons[j].pose().ori_.X();
				msg.pose.pose.orientation.y = cartons[j].pose().ori_.Y();
				msg.pose.pose.orientation.z = cartons[j].pose().ori_.Z();
				msg.pose.pose.orientation.w = cartons[j].pose().ori_.W();
				
				// todo: why in this weird order??????????????????
//				msg.bounding_box_lwh.x = cartons[j].bb_in_pose().sizes()(2);
//				msg.bounding_box_lwh.y = cartons[j].bb_in_pose().sizes()(0);
//				msg.bounding_box_lwh.z = cartons[j].bb_in_pose().sizes()(1);
				msg.bounding_box_lwh.x = cartons[j].bb_in_pose().sizes()(0);
				msg.bounding_box_lwh.y = cartons[j].bb_in_pose().sizes()(1);
				msg.bounding_box_lwh.z = cartons[j].bb_in_pose().sizes()(2);
				
				array.detections.push_back(msg);
			}
		}
		
		pub_cartons_.publish(array);
	}
	
	void publish_scan(const std_msgs::Header &header) {
		if(pub_scan_.getNumSubscribers()<1) return;
		assert(classifier_floor_);
		
		sensor_msgs::LaserScan msg;
		msg.header = header;
		msg.angle_min = 0;
		msg.angle_max = 2*M_PI;
		msg.angle_increment = 2*M_PI/classifier_floor_->get_rays().size();
		msg.range_min = 0;
		msg.range_max = classifier_floor_->max_range();
		msg.ranges.resize(classifier_floor_->get_rays().size());
		std::copy(classifier_floor_->get_rays().begin(), classifier_floor_->get_rays().end(), msg.ranges.begin());
		
		pub_scan_.publish(msg);
	}

	void cb_camera_info(const sensor_msgs::CameraInfoConstPtr& camera_info)
	{
	  ROS_INFO("camera_info");
	  
	  camera_info_ = *camera_info;
	  sub_camera_info_.shutdown();	//just get one message (assume camera does not change while running)
	  
	  Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
	  std::copy(camera_info_.P.begin(), camera_info_.P.end(), P.data());
	  P = P.transpose().eval();
	  std::cout<<"P\n"<<P<<std::endl;
	  std::cout<<"Pi\n"<<P.inverse()<<std::endl;
	  P.row(0) /= camera_info_.width;
	  P.row(1) /= camera_info_.height;
	  
	  init_context();
	  
	  ctxt_->set_projection(P.topLeftCorner<3,3>());
	  
	  start();
	}
	
	typedef message_filters::sync_policies::ApproximateTime<cob_3d_mapping_msgs::PlaneScene, sensor_msgs::Image> TSyncPolicy;
	
	boost::shared_ptr<message_filters::Subscriber<cob_3d_mapping_msgs::PlaneScene> > sub_scene_;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_col_img_;
	boost::shared_ptr<message_filters::Synchronizer<TSyncPolicy> > sync_;
	
	ros::ServiceServer reset_server_;
	
	ros::Subscriber sub_scene2_, sub_camera_info_;
	ros::Publisher pub_scan_, pub_cartons_;
	
	sensor_msgs::CameraInfo camera_info_;
	
	tf::TransformListener tf_listener_;
	
	void init_context() {
		ros::NodeHandle pn("~");
		
		ctxt_.reset(new cob_3d_geometry_map::GlobalContext);
		//register default classifiers
		double floor_height=0;
		
		pn.param<bool>("merge_enabled", ctxt_->merge_enabled(), true);
		
		pn.param<double>("floor_height", floor_height, floor_height);
		
		Eigen::Vector3f floor_offset(0,floor_height,0);
		Eigen::Vector3f floor_orientation = Eigen::Vector3f::UnitY();
		
		ctxt_->registerClassifier(classifier_floor_ = new cob_3d_geometry_map::DefaultClassifier::Classifier_Floor(floor_orientation, floor_offset, 0.1f, 0.1f, 128));
		
		double carton_tolerance_left_right = 0.0;
		double carton_tolerance_top = 0.0;
		
		double min_coverage_seeing=0.8, min_coverage_expecting=0.8, bandwith_orientation=0.2;
		
		pn.param<double>("carton_min_coverage_seeing", min_coverage_seeing, min_coverage_seeing);
		pn.param<double>("carton_min_coverage_expecting", min_coverage_expecting, min_coverage_expecting);
		pn.param<double>("carton_bandwith_orientation", bandwith_orientation, bandwith_orientation);
		
		XmlRpc::XmlRpcValue cartonList;
		if (nh_.getParam("/item_list", cartonList))
		{
		  for(int i=0; i<cartonList.size(); i++)
		  {
			  SCarton carton(cartonList[i]);
			  
			  for(int n=0; n<carton.box_count_horizontal_; n++)
			  {
			
				Eigen::Vector3f carton_offset = carton.pos_.cast<float>();
				Eigen::Vector3f carton_orientation = Eigen::Vector3f::UnitY();
				Eigen::Vector3f carton_size = carton.dim_.cast<float>();
					
				//generate virtual boxes
				carton_size(0)   /= carton.box_count_horizontal_;
				carton_offset(0) += n*carton_size(0);
				
				carton_offset(1) -= 0.033;
				
				carton_offset(0) -= carton_tolerance_left_right;
				carton_size(0) += 2*carton_tolerance_left_right;
				
				carton_offset(1) -= carton_tolerance_top;
				carton_size(1) += carton_tolerance_top;
				
				std::vector<double> widths;
				widths.push_back(carton.dim_(0)/carton.box_count_horizontal_);
				
				cob_3d_geometry_map::CustomClassifier::Classifier_Carton *carton_front, *carton_side;
				ctxt_->registerClassifier( carton_front=new cob_3d_geometry_map::CustomClassifier::Classifier_Carton(carton.id_+n*100, Eigen::AngleAxisf(0,carton_orientation)*Eigen::Translation3f(carton_offset), carton_size, widths, min_coverage_seeing, min_coverage_expecting, bandwith_orientation));
				ctxt_->registerClassifier( carton_side=new cob_3d_geometry_map::CustomClassifier::Classifier_Carton(carton_front) );
				classifier_cartons_.push_back(carton_side);
			  }
				
			  ROS_INFO("added carton %d  (%d x)", carton.id_, carton.box_count_horizontal_);
		  }
		}
	}
	
	void start() {
		//load params
		ros::NodeHandle pn("~");
		pn.param<std::string>("target_frame", target_frame_, "");
		
		ROS_INFO("loaded parameters:");
		ROS_INFO("target_frame:\t%s", target_frame_.c_str());
		
		cob_3d_visualization::RvizMarkerManager::get().createTopic("marker").setFrameId(target_frame_);//.clearOld();
		 
		//now start the ROS stuff
		//sub_scene_.reset(new message_filters::Subscriber<cob_3d_mapping_msgs::PlaneScene>(nh_, "scene", 1));
		//sub_col_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "color_image", 1));
		
		sub_scene2_ = nh_.subscribe("scene", 1, &GeometryNode::callback2, this);
		 
		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
		//sync_.reset(new message_filters::Synchronizer<TSyncPolicy>(TSyncPolicy(10), *sub_scene_, *sub_col_img_));
		//sync_->registerCallback(boost::bind(&GeometryNode::callback, this, _1, _2));
	}
	
	void reset() {
		sub_scene2_.shutdown();
		if(sub_scene_) sub_scene_->unsubscribe();
		if(sub_col_img_) sub_col_img_->unsubscribe();
		
		/*sub_scene_.reset();
		sub_col_img_.reset();
		sync_.reset();*/
		
		classifier_floor_ = NULL;
		ctxt_.reset();
		classifier_cartons_.clear();
		
		sub_camera_info_ = nh_.subscribe("camera_info", 1, &GeometryNode::cb_camera_info, this);
	}
	
	virtual bool register_scene(const cob_3d_geometry_map::Context::ConstPtr &new_scene, const cob_3d_geometry_map::Context::ConstPtr &map, nuklei::kernel::se3 &tf_out) {
		tf_out = cob_3d_geometry_map::cast(tf2target_);
		return true;
	}
	
public:

	GeometryNode() : classifier_floor_(NULL), tf2target_(Eigen::Translation3d(0,0,0)) {
		ros::NodeHandle pn("~");
		
		sub_camera_info_ = nh_.subscribe("camera_info", 1, &GeometryNode::cb_camera_info, this);
		pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
		pub_cartons_ = nh_.advertise<cob_object_detection_msgs::DetectionArray>("cartons", 10);
		reset_server_ = pn.advertiseService("reset", &GeometryNode::reset, this);
	}

	/**
	* @brief resets complete map
	*
	* resets complete map
	*
	* @param req not needed
	* @param res not needed
	*
	* @return nothing
	*/
	bool
#if ROS_VERSION_MINIMUM(1, 11, 0)
	reset(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res)
#else
	reset(cob_srvs::Trigger::Request &req,
		cob_srvs::Trigger::Response &res)
#endif
	{
		//TODO: add mutex
		ROS_INFO("Resetting...");
		reset();
#if ROS_VERSION_MINIMUM(1, 11, 0)
		res.success = true;
#else
		res.success.data = true;
#endif
		
		return true;
	}


};
