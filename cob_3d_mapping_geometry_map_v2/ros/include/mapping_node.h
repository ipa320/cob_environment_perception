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

#include <cob_srvs/Trigger.h>

class GeometryNode {
	ros::NodeHandle nh_;
	cob_3d_geometry_map::GlobalContext::Ptr ctxt_;
	cob_3d_geometry_map::DefaultClassifier::Classifier_Floor *classifier_floor_;

	void callback(const cob_3d_mapping_msgs::PlaneSceneConstPtr& scene, const sensor_msgs::ImageConstPtr& color_img)
	{
	  ROS_INFO("callback");
	  
	  ctxt_->add_scene(*scene);
	}

	void callback2(const cob_3d_mapping_msgs::PlaneSceneConstPtr& scene)
	{
	  ROS_INFO("callback2");
	  
	  cob_3d_visualization::RvizMarkerManager::get().setFrameId(scene->header.frame_id);
	  
	  ctxt_->add_scene(*scene);
	  ctxt_->visualize_markers();
	  
	  publish_scan(scene->header);
	  
	  system("read x");
	}
	
	void publish_scan(const std_msgs::Header &header) {
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
	  
	  Eigen::Matrix3d P;
	  std::copy(camera_info_.K.begin(), camera_info_.K.end(), P.data());
	  P = P.transpose().eval();
	  std::cout<<"P\n"<<P<<std::endl;
	  std::cout<<"Pi\n"<<P.inverse()<<std::endl;
	  P.row(0) /= camera_info_.width;
	  P.row(1) /= camera_info_.height;
	  
	  init_context();
	  
	  ctxt_->set_projection(P);
	  
	  start();
	}
	
	typedef message_filters::sync_policies::ApproximateTime<cob_3d_mapping_msgs::PlaneScene, sensor_msgs::Image> TSyncPolicy;
	
	boost::shared_ptr<message_filters::Subscriber<cob_3d_mapping_msgs::PlaneScene> > sub_scene_;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_col_img_;
	boost::shared_ptr<message_filters::Synchronizer<TSyncPolicy> > sync_;
	
	ros::ServiceServer reset_server_;
	
	ros::Subscriber sub_scene2_, sub_camera_info_;
	ros::Publisher pub_scan_;
	
	sensor_msgs::CameraInfo camera_info_;
	
	void init_context() {
		ctxt_.reset(new cob_3d_geometry_map::GlobalContext);
		//register default classifiers
		
		Eigen::Vector3f floor_offset(0,0.5f,0);
		Eigen::Vector3f floor_orientation = Eigen::Vector3f::UnitY();
		
		ctxt_->registerClassifier(classifier_floor_ = new cob_3d_geometry_map::DefaultClassifier::Classifier_Floor(floor_orientation, floor_offset, 0.1f, 0.1f, 128));
		
		/*XmlRpc::XmlRpcValue topicList;
		std::vector<std::string> topics;

		if (private_nh.getParam("region_of_in_terests", topicList))
		{
		  std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
		  for (i = topicList.begin(); i != topicList.end(); i++)
		  {
			std::string topic_name;
			std::string topic_type;

			topic_name = i->first;
			topic_type.assign(i->second["topic_type"]);

			topics.push_back(topic_name);
		  }
		}*/
		
		Eigen::Vector3f carton_offset(-0.1,0.05f,1.19);
		Eigen::Vector3f carton_orientation = Eigen::Vector3f::UnitY();
		
		std::vector<double> widths;
		widths.push_back(0.2);
		widths.push_back(0.25);
		widths.push_back(0.3);
		cob_3d_geometry_map::CustomClassifier::Classifier_Carton *carton_front;
		ctxt_->registerClassifier( carton_front=new cob_3d_geometry_map::CustomClassifier::Classifier_Carton(Eigen::AngleAxisf(0,carton_orientation)*Eigen::Translation3f(carton_offset), Eigen::Vector3f(0.4,0.25, 0.2), widths) );
		ctxt_->registerClassifier( new cob_3d_geometry_map::CustomClassifier::Classifier_Carton(carton_front) );
	}
	
	void start() {		
		//now start the ROS stuff
		sub_scene_.reset(new message_filters::Subscriber<cob_3d_mapping_msgs::PlaneScene>(nh_, "scene", 1));
		sub_col_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "color_image", 1));
		
		sub_scene2_ = nh_.subscribe("scene", 1, &GeometryNode::callback2, this);
		 
		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
		sync_.reset(new message_filters::Synchronizer<TSyncPolicy>(TSyncPolicy(10), *sub_scene_, *sub_col_img_));
		sync_->registerCallback(boost::bind(&GeometryNode::callback, this, _1, _2));
	}
	
	void reset() {
		sub_scene2_.shutdown();
		if(sub_scene_) sub_scene_->unsubscribe();
		if(sub_col_img_) sub_col_img_->unsubscribe();
		
		sub_scene_.reset();
		sub_col_img_.reset();
		sync_.reset();
		
		classifier_floor_ = NULL;
		ctxt_.reset();
		
		sub_camera_info_ = nh_.subscribe("camera_info", 1, &GeometryNode::cb_camera_info, this);
	}
	
public:

	GeometryNode() : classifier_floor_(NULL) {
		sub_camera_info_ = nh_.subscribe("camera_info", 1, &GeometryNode::cb_camera_info, this);
		pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
		reset_server_ = nh_.advertiseService("reset", &GeometryNode::reset, this);
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
	reset(cob_srvs::Trigger::Request &req,
		cob_srvs::Trigger::Response &res)
	{
		//TODO: add mutex
		ROS_INFO("Resetting...");
		reset();
		return true;
	}


};
