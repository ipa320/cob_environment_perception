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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_vision
 * \note
 *  ROS package name: dynamic_tutorials
 *
 * \author
 *  Author: goa-jh
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: Nov 7, 2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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

#define HAS_RGB 1

//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/point_types.h>
#define PCL_MINOR (PCL_VERSION[2] - '0')

#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include "cob_3d_registration/RegistrationPCD.h"
#include "cob_3d_registration/Parameterlist.h"
#include "cob_3d_registration/EvaluationResult.h"
#include "parameters/parameters_bag.h"

#include <sensor_msgs/CameraInfo.h>

#include <cob_3d_registration/registration_icp.h>

#include <vtkCommand.h>
#include <pcl/features/feature.h>
#ifndef GICP_ENABLE
#include <cob_3d_registration/registration_icp_moments.h>
#include <cob_3d_registration/registration_icp_fpfh.h>
#include <cob_3d_registration/registration_icp_narf.h>

//#include <registration/registration_fastslam.h>
//#include <cob_vision_utils/VisionUtils.h>
#include <opencv2/opencv.hpp>

#include <cob_3d_registration/registration_icp_edges.h>
#include <cob_3d_registration/registration_info.h>
//#include <cob_3d_registration/registration_correspondence.h>
#endif

#include <pcl/filters/voxel_grid.h>
#include <cob_3d_registration/preprocessing/kinect_error.h>

#include <cv_bridge/cv_bridge.h>
#include <sys/stat.h>

#include <cob_3d_registration/measurements/measure.h>
#include <std_srvs/Trigger.h>
#include <cob_3d_mapping_msgs/TriggerAction.h>

#ifdef PCL_VERSION_COMPARE
  #include <pcl/point_traits.h>
  #include <pcl/kdtree/kdtree_flann.h>
  #include <pcl/common/eigen.h>
  #include <pcl/registration/correspondence_estimation.h>
#else
  #include <pcl/ros/point_traits.h>
#endif






using namespace tf;
#define SHOW_MAP 0

/// helper class to measure maximum memory usage over time
class MemoryOperator
{
  measurement_tools::MemoryUsage mu_;
  int max_used_;
  bool run_, running_;
public:
  MemoryOperator():
    max_used_(0), run_(true), running_(false)
  {
  }

  void halt() {
    run_=false;
  }

  void wait() {
    ROS_INFO("halt");
    while(running_) run_=false;
    ROS_INFO("halt");
  }

  int getKB() {
    return max_used_;
  }

  void run() {
    running_ = true;
    while(run_) {
      int m = mu_.getKB();
      max_used_ = std::max(m, max_used_);
      usleep(10000);
    }
    running_=false;
  }
};

//####################
//#### node class ####
class RegistrationNodelet : public nodelet::Nodelet
{
#if HAS_RGB
  typedef pcl::PointXYZRGB Point;
#else
  typedef pcl::PointXYZ Point;
#endif

public:
  // Constructor
  RegistrationNodelet():
    ctr_(0), reg_(NULL), is_running_(false)
  {
    //onInit();
  }

  RegistrationNodelet(bool dummy):
    ctr_(0), reg_(NULL), is_running_(false)
  {
  }


  // Destructor
  ~RegistrationNodelet()
  {
    /// void
    delete reg_;
  }

  /**
   * @brief initializes parameters
   *
   * initializes parameters
   *
   * @return nothing
   */
  void
  onInit()
  {
    //PCLNodelet::onInit();
    parameters_.setNodeHandle(n_);
    n_ = getNodeHandle();

    parameters_.addParameter("world_frame_id");
    if(!parameters_.getParam("world_frame_id",world_id_))
      world_id_="/map"; //default value

    parameters_.addParameter("corrected_frame_id");
    if(!parameters_.getParam("corrected_frame_id",corrected_id_))
      corrected_id_="/camera_registered"; //default value

    //general
    parameters_.addParameter("algo");
    parameters_.addParameter("voxelsize");

    //icp
    parameters_.addParameter("max_iterations");
    parameters_.addParameter("corr_dist");
    parameters_.addParameter("trf_epsilon");
    parameters_.addParameter("outlier_rejection_threshold");
    parameters_.addParameter("use_only_last_refrence");


    //icp moments
    parameters_.addParameter("moments_radius");

    //icp fpfh
    parameters_.addParameter("fpfh_radius");

    //icp edges
    parameters_.addParameter("edge_radius");
    parameters_.addParameter("threshold");
    parameters_.addParameter("distance_threshold");

    //info
    parameters_.addParameter("use_icp");
    parameters_.addParameter("threshold_diff");
    parameters_.addParameter("threshold_step");
    parameters_.addParameter("min_info");
    parameters_.addParameter("max_info");
    parameters_.addParameter("always_relevant_changes");

    reset_server_ = n_.advertiseService("reset", &RegistrationNodelet::reset, this);
    camera_info_sub_ = n_.subscribe("camera_info", 1, &RegistrationNodelet::cameraInfoSubCallback, this);
    point_cloud_pub_aligned_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_aligned",1);
    keyframe_trigger_server_ = n_.advertiseService("trigger_keyframe", &RegistrationNodelet::onKeyframeCallback, this);
    as_= boost::shared_ptr<actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerAction> >(new actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerAction>(n_, "trigger_mapping", boost::bind(&RegistrationNodelet::actionCallback, this, _1), false));
    as_->start();
    //point_cloud_pub_ = n_.advertise<pcl::PointCloud<Point> >("result_pc",1);
    marker_pub_ = n_.advertise<pcl::PointCloud<Point> >("result_marker",1);
    //point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &RegistrationNodelet::pointCloudSubCallback, this);
    //register_ser_ = n_.advertiseService("registration_process", &RegistrationNodelet::registerService, this);
    //marker_pub_ = n_.advertise<visualization_msgs::Marker>("markers", 1);

    //after reading params
    buildAlgo();

    //say what we are doing here
    //publishParameters();
  }

  /**
   * @brief resetes transformation
   *
   * resetes transformation
   *
   * @param req not needed
   * @param res not needed
   *
   * @return nothing
   */
  bool
  reset(std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res)
  {
    //TODO: add mutex
    ROS_INFO("Resetting transformation...");
    if(reg_) buildAlgo();
    return true;
  }

  double _time_;
  /**
   * @brief callback for point cloud subroutine
   *
   * callback for point cloud subroutine which stores the point cloud
   * for further calculation
   *
   * @param pc_in  new point cloud
   *
   * @return nothing
   */
  void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    pc_frame_id_=pc_in->header.frame_id;

    if(reg_==0 || pc_in==0 || pc_in->size()<1 || !is_running_)
      return;

    reg_->setInputOginalCloud(pc_in);

    /*StampedTransform transform, transform2;
    try
    {
      tf_listener_.waitForTransform(pc_frame_id_, world_id_, pc_in->header.stamp, ros::Duration(0.1));
      tf_listener_.lookupTransform(pc_frame_id_, world_id_, pc_in->header.stamp, transform);

      ROS_DEBUG("Registering new point cloud");

      Eigen::Affine3d af;
      tf::TransformTFToEigen(transform, af);
      ROS_INFO("got odometry");
      _time_ = transform.stamp_.toSec();
      reg_->setOdometry(af.matrix().cast<float>());
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[registration] : %s",ex.what());
      return;
    }

    ROS_INFO("ts diff %f",pc_in->header.stamp.toSec()-_time_);*/

    if(do_register(*pc_in,cv_bridge::CvImagePtr(),NULL)||ctr_==0) {
      if(point_cloud_pub_aligned_.getNumSubscribers()>0) {
        pcl::PointCloud<Point> pc;
        pc.header.frame_id = pc_in->header.frame_id;
        pcl::transformPointCloud(*pc_in,pc,reg_->getTransformation());
        point_cloud_pub_aligned_.publish(pc);
      }

      StampedTransform transform;
      Eigen::Affine3d af;
      af.matrix()=reg_->getTransformation().cast<double>();
      //std::cout<<reg_->getTransformation()<<"\n";
      tf::transformEigenToTF(af,transform);
      //std::cout << transform.stamp_ << std::endl;
      std_msgs::Header header;
      pcl_conversions::fromPCL(pc_in->header, header);
      tf_br_.sendTransform(tf::StampedTransform(transform, header.stamp, world_id_, corrected_id_));

      ROS_WARN("registration successful");
    }
    else
      ROS_WARN("registration not successful");
    ctr_++;

    if(marker_pub_.getNumSubscribers()>0)
    {
      std::string s_algo;
      if(parameters_.getParam("algo",s_algo) && s_algo=="info") {
        pcl::PointCloud<Point> result = *((cob_3d_registration::Registration_Infobased<Point>*)reg_)->getMarkers2();
        result.header=pc_in->header;
        marker_pub_.publish(result);
      }
    }
  }

  /**
   * @brief callback for point cloud subroutine
   *
   * callback for keyframe subroutine which loads in the first step
   * the unexact transformation from the laser sensors and calibrates the
   * input cloud from the 3d camera. This already transformed data will be
   * used to build a 3d map either aligned to the first frame or to an
   * existing map. Additionally debug output to *.pcd files are possible.
   *
   * @param req  not used
   * @param res  not used
   *
   * @return nothing
   */
  bool onKeyframeCallback(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res)
  {
    res.success = false;

    if(reg_==0 || pc_frame_id_.size()<1 || !is_running_)
      return true;

    StampedTransform transform;
    try
    {
      std::stringstream ss2;
      tf_listener_.waitForTransform(world_id_, pc_frame_id_, ros::Time(0), ros::Duration(0.1));
      tf_listener_.lookupTransform(world_id_, pc_frame_id_, ros::Time(0), transform);

      ROS_DEBUG("Registering new point cloud");

      Eigen::Affine3d af;
      tf::transformTFToEigen(transform, af);
      reg_->setOdometry(af.matrix().cast<float>());
      reg_->setMoved(true);

    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[registration] : %s",ex.what());
      return false;
    }

    res.success = true;
    return true;
  }

  /**
   * @brief action callback
   *
   * default action callback to start or stop registration
   *
   * @param goal settings
   *
   * @return nothing
   */
  void
  actionCallback(const cob_3d_mapping_msgs::TriggerGoalConstPtr &goal)
  {
    cob_3d_mapping_msgs::TriggerResult result;
    if(goal->start && !is_running_)
    {
      ROS_INFO("Starting mapping...");
      point_cloud_sub_ = n_.subscribe("point_cloud2", 1, &RegistrationNodelet::pointCloudSubCallback, this);
      //point_cloud_sub_.subscribe(n_, "point_cloud2", 1);
      //transform_sub_.subscribe(n_, "transform_reg", 1);
      is_running_ = true;
    }
    else if(!goal->start && is_running_)
    {
      ROS_INFO("Stopping mapping...");
      point_cloud_sub_.shutdown();//unsubscribe();
      //transform_sub_.unsubscribe();
      //first_ = true;
      is_running_ = false;
    }
    as_->setSucceeded(result);
  }

  /**
   * creating parameter settings for evaluation
   * it's not thought to be used for registration self
   */
  void createParamters(const std::string &algo) {

    //for all icp, fastslam
    if(algo.find("icp")==0 || algo.find("gicp")==0 || algo=="fastslam") {

      //TODO: not with all!!!!
      parameters_.createParam(n_,"voxelsize",0.04).setMin(0.04).setMax(0.041).setStep(10.02); //0cm or 2cm
    }

    //icp
    if(algo.find("icp")==0 || algo.find("gicp")==0) {
      parameters_.createParam(n_,"max_iterations",40).setMin(40).setMax(41).setStep(30);
      parameters_.createParam(n_,"corr_dist",0.2).setMin(0.2).setMax(0.3).setStep(0.2);
      parameters_.createParam(n_,"trf_epsilon",3.5).setMin(3.5).setMax(3.6).setStep(1.); //10^-(1...10)
      parameters_.createParam(n_,"outlier_rejection_threshold",0.1).setMax(0.3).setStep(0.1);
      parameters_.createParam(n_,"use_only_last_refrence",1).setMax(1).setStep(2); //true/false
    }

    //icp moments
    if(algo=="icp_moments") {
      parameters_.setParam("voxelsize_min",0.04);
      parameters_.createParam(n_,"moments_radius",0.05).setMax(0.2).setStep(0.05);
    }

    //icp fpfh
    if(algo=="icp_fpfh") {
      parameters_.setParam("voxelsize_min",0.04);
      parameters_.createParam(n_,"fpfh_radius",0.05).setMax(0.2).setStep(0.05);
    }

    //icp edges
    if(algo=="icp_edges") {
      //parameters_.createParam(n_,"edge_radius",0.01).setMax(0.2).setStep(0.01);
      parameters_.createParam(n_,"threshold",0.45).setMin(0.45).setMax(0.456).setStep(0.1);
      //parameters_.createParam(n_,"distance_threshold",0.01).setMax(0.2).setStep(0.01);
    }

    //info
    if(algo=="info") {
      parameters_.createParam(n_,"use_icp",1).setMax(1).setStep(1); //true/false
    }
  }

  /**
   * reading out all parameters from parameter bag and publishing them as parameterlist
   * thought to be used for evaluation
   */
  void publishParameters() {
    ros::ServiceClient client = n_.serviceClient< ::cob_3d_registration::Parameterlist>("paramterlist");
    ::cob_3d_registration::Parameterlist srv;

    const std::map<std::string, int> &is = parameters_.getInts();
    for(std::map<std::string, int>::const_iterator it=is.begin(); it!=is.end(); it++) {
      std::ostringstream s;
      s<<it->second;
      srv.request.types.push_back("int");
      srv.request.names.push_back(it->first);
      srv.request.values.push_back(s.str());
    }

    const std::map<std::string, double> &ds = parameters_.getDoubles();
    for(std::map<std::string, double>::const_iterator it=ds.begin(); it!=ds.end(); it++) {
      std::ostringstream s;
      s<<it->second;
      srv.request.types.push_back("double");
      srv.request.names.push_back(it->first);
      srv.request.values.push_back(s.str());
    }

    const std::map<std::string, std::string> &ss = parameters_.getStrings();
    for(std::map<std::string, std::string>::const_iterator it=ss.begin(); it!=ss.end(); it++) {
      std::ostringstream s;
      s<<it->second;
      srv.request.types.push_back("string");
      srv.request.names.push_back(it->first);
      srv.request.values.push_back(s.str());
    }

    bool b=false;
    int tries=0;
    do {
      b=client.call(srv);
      if(!b) sleep(2);
      ++tries;
    } while(!b && tries<5);

    if (b)
    {
      ROS_DEBUG("published paramters successfully");
    }
    else
    {
      ROS_ERROR("Failed to call paramterlist service");
    }
  }

  /**
   * reading out parameter and creating an instance of the algorithm
   * setting all needed parameters for algorithm
   */
  void buildAlgo()
  {
    delete reg_;
    reg_=NULL;

    //get algo
    int e_algo=E_ALGO_ICP_EDGES;

    //parameter "algo" determines the algorithm
    std::string s_algo;
    if(parameters_.getParam("algo",s_algo)) {
      if(s_algo=="icp")
        e_algo=E_ALGO_ICP;
      else if(s_algo=="icp lm")
        e_algo=E_ALGO_ICP_LM;
      else if(s_algo=="gicp")
        e_algo=E_ALGO_GICP;
      else if(s_algo=="icp_moments")
        e_algo=E_ALGO_ICP_MOMENTS;
      else if(s_algo=="icp_fpfh")
        e_algo=E_ALGO_ICP_FPFH;
      else if(s_algo=="icp_narf")
        e_algo=E_ALGO_ICP_NARF;
      //      else if(s_algo=="fastslam")
      //        e_algo=E_ALGO_FASTSLAM;
      else if(s_algo=="icp_edges")
        e_algo=E_ALGO_ICP_EDGES;
      else if(s_algo=="info")
        e_algo=E_ALGO_INFO;
      else if(s_algo=="cor")
        e_algo=E_ALGO_COR;
      else
        ROS_WARN("algo %s not found", s_algo.c_str());

    }
    else
      ROS_WARN("using algo icp");

    //init: settings
    ROS_INFO("init %s", s_algo.c_str());
    switch(e_algo) {
      case E_ALGO_ICP:
        reg_ = new cob_3d_registration::Registration_ICP<Point>();

        setSettings_ICP((cob_3d_registration::Registration_ICP<Point>*)reg_);
        break;

      case E_ALGO_GICP:
        reg_ = new cob_3d_registration::Registration_ICP<Point>();

        setSettings_ICP((cob_3d_registration::Registration_ICP<Point>*)reg_);
        ((cob_3d_registration::Registration_ICP<Point>*)reg_)->setUseGICP(true);

        break;

      case E_ALGO_ICP_LM:
        reg_ = new cob_3d_registration::Registration_ICP<Point>();
        ((cob_3d_registration::Registration_ICP<Point>*)reg_)->setNonLinear(true);

        setSettings_ICP((cob_3d_registration::Registration_ICP<Point>*)reg_);
        break;

      case E_ALGO_ICP_MOMENTS:
#ifndef GICP_ENABLE
        reg_ = new cob_3d_registration::Registration_ICP_Moments<Point>();

        setSettings_ICP_Moments((cob_3d_registration::Registration_ICP_Moments<Point>*)reg_);
#else
        ROS_ERROR("not supported");
#endif
        break;

      case E_ALGO_ICP_FPFH:
#ifndef GICP_ENABLE
        reg_ = new cob_3d_registration::Registration_ICP_FPFH<Point>();

        setSettings_ICP_FPFH((cob_3d_registration::Registration_ICP_FPFH<Point>*)reg_);
#else
        ROS_ERROR("not supported");
#endif
        break;

      case E_ALGO_ICP_NARF:
#ifndef GICP_ENABLE
        reg_ = new cob_3d_registration::Registration_ICP_NARF<Point>();

        setSettings_ICP_NARF((cob_3d_registration::Registration_ICP_NARF<Point>*)reg_);
#else
        ROS_ERROR("not supported");
#endif
        break;

      case E_ALGO_ICP_EDGES:
#ifndef GICP_ENABLE
#if HAS_RGB
        reg_ = new cob_3d_registration::Registration_ICP_Edges<Point>();

        setSettings_ICP_Edges((cob_3d_registration::Registration_ICP_Edges<Point>*)reg_);
#endif
#else
        ROS_ERROR("not supported");
#endif
        break;

        /*case E_ALGO_FASTSLAM:
#ifndef GICP_ENABLE
        reg_ = new cob_3d_registration::Registration_FastSLAM<Point>();

        //setSettings_ICP_FastSLAM((cob_3d_registration::Registration_FastSLAM<Point>*)reg_);
#else
        ROS_ERROR("not supported");
#endif
        break;*/

      case E_ALGO_INFO:
#ifndef GICP_ENABLE
        reg_ = new cob_3d_registration::Registration_Infobased<Point>();

        setSettings_Info((cob_3d_registration::Registration_Infobased<Point>*)reg_);
#else
        ROS_ERROR("not supported");
#endif
        break;

      case E_ALGO_COR:
//#ifndef GICP_ENABLE
#if 0
        reg_ = new cob_3d_registration::Registration_Corrospondence<Point>();

        //((Registration_Corrospondence<Point>*)reg_)->setKeypoints(new Keypoints_Segments<Point>);
        //((Registration_Corrospondence<Point>*)reg_)->setKeypoints(new Keypoints_Narf<Point>);

        //setSettings_Cor((cob_3d_registration::Registration_Corrospondence<Point>*)reg_);
#else
        ROS_ERROR("not supported");
#endif
        break;

    }
    ROS_INFO("done");

  }

  /**
   * service callback to register a dataset
   * returning success
   */
  bool registerService(::cob_3d_registration::RegistrationPCD::Request  &req,
                       ::cob_3d_registration::RegistrationPCD::Response &res )
  {
    ROS_INFO("register...");


    pcl::PointCloud<Point> pc;
    pcl::io::loadPCDFile(req.pcd_fn, pc);

    for(int i=0; i<(int)pc.size(); i++) {
      if(pc[i].z==0||pc[i].z>10)
        pc[i].z=pc[i].y=pc[i].x=std::numeric_limits<float>::quiet_NaN();
    }

    sensor_msgs::Image img;
//    {
//      FILE *fp = fopen(req.img_fn.c_str(), "rb");
//      if(!fp) return false;
//
//      struct stat filestatus;
//      stat(req.img_fn.c_str(), &filestatus );
//
//      uint8_t *up = new uint8_t[filestatus.st_size];
//      fread(up,filestatus.st_size,1,fp);
//      img.deserialize(up);
//      delete up;
//
//      fclose(fp);
//    }

    cv::Mat img_depth(pc.height, pc.width, CV_16UC1);
#ifdef USE_DEPTH_IMG_
    sensor_msgs::Image img_depth;
    {
      FILE *fp = fopen(req.depth_img_fn.c_str(), "rb");
      if(!fp) return false;

      struct stat filestatus;
      stat(req.img_fn.c_str(), &filestatus );

      uint8_t *up = new uint8_t[filestatus.st_size];
      fread(up,filestatus.st_size,1,fp);
      img_depth.deserialize(up);
      delete up;

      fclose(fp);
    }
#elif 0
    cv::Mat cv_pc(pc.height, pc.width, CV_32FC1);
    for(int x=0; x<pc.width; x++)
      for(int y=0; y<pc.height; y++)
        *(cv_pc.ptr<float>(y)+x) = pc.points[x+y*pc.width].z;

    ipa_Utils::ConvertToShowImage(cv_pc, img_depth, 1, 0., 5.);
#else
    for(int x=0; x<(int)pc.width; x++)
      for(int y=0; y<(int)pc.height; y++) {
        int raw_depth = std::max(0, (int)(((1./pc.points[x+y*pc.width].z)-3.3309495161)/-0.0030711016));
        *(img_depth.ptr<unsigned short>(y)+x) = raw_depth<2048?raw_depth:0;
      }
#endif

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, "rgb8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }

    cv_bridge::CvImagePtr cv_ptr_depth;
#ifdef USE_DEPTH_IMG_
    try
    {
      cv_ptr_depth = cv_bridge::toCvCopy(img_depth, "rgb8");
      img_depth = cv_ptr_depth->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }
#endif

    reg_->setInputOginalCloud(boost::shared_ptr<const pcl::PointCloud<Point> >(new pcl::PointCloud<Point>(pc)));

    measurement_tools::PrecisionStopWatchAll psw;
    do_preprocess(pc,cv_ptr,img_depth);
    ROS_INFO("do_preprocess took %f", psw.precisionStop());
    psw.precisionStart();
    do_register(pc,cv_ptr,&img_depth);
    ROS_INFO("do_register took %f", psw.precisionStop());

    publish_results();

    //debug purpose
#if SHOW_MAP

    if(marker_pub_.getNumSubscribers()&&reg_->getMarkers()) {
      for(int i=0; i<reg_->getMarkers()->size(); i++)
#if HAS_RGB
        publishMarkerPoint(reg_->getMarkers()->points[i], i, reg_->getMarkers()->points[i].r/255., reg_->getMarkers()->points[i].g/255., reg_->getMarkers()->points[i].b/255.);
#else
      publishMarkerPoint(reg_->getMarkers()->points[i], i, 1,0,0);
#endif
    }

#ifndef GICP_ENABLE
    std::string s_algo;
    if(parameters_.getParam("algo",s_algo) && s_algo=="info") {
      pcl::PointCloud<Point> result = *((cob_3d_registration::Registration_Infobased<Point>*)reg_)->getMarkers2();
      result.header.frame_id="/head_cam3d_frame";
      marker2_pub_.publish(result);

      for(int i=0; i<((cob_3d_registration::Registration_Infobased<Point>*)reg_)->getSource().size(); i++)
        publishLineMarker( ((cob_3d_registration::Registration_Infobased<Point>*)reg_)->getSource().points[i].getVector3fMap(), ((cob_3d_registration::Registration_Infobased<Point>*)reg_)->getTarget().points[i].getVector3fMap(), -i);
    }
    else if(parameters_.getParam("algo",s_algo) && s_algo=="cor") {
#ifdef PCL_VERSION_COMPARE
      pcl::Correspondences cor;
#else
      pcl::registration::Correspondences cor;
#endif
      ((Registration_Corrospondence<Point>*)reg_)->getKeypoints()->getCorrespondences(cor);
      for(int i=0; i<cor.size(); i++)
        publishLineMarker( ((cob_3d_registration::Registration_Corrospondence<Point>*)reg_)->getKeypoints()->getSourcePoints()->points[cor[i].indexQuery].getVector3fMap(), ((cob_3d_registration::Registration_Corrospondence<Point>*)reg_)->getKeypoints()->getTargetPoints()->points[cor[i].indexMatch].getVector3fMap(), -i);
    }
#endif

    if(registration_result_) {
      pcl::PointCloud<Point> result;
      pcl::io::loadPCDFile(req.pcd_fn, result);
      static pcl::PointCloud<Point> map;
      pcl::transformPointCloud(result,result,reg_->getTransformation());
      map.header.frame_id = result.header.frame_id;
      map += result;

      pcl::VoxelGrid<Point> voxel;
      voxel.setInputCloud(map.makeShared());
      float voxelsize=0.04;
      voxel.setLeafSize(voxelsize,voxelsize,voxelsize);
      voxel.filter(map);

      map.header.frame_id="/head_cam3d_link";
      point_cloud_pub_.publish(map);

      {
        std::string s_algo;
        parameters_.getParam("algo",s_algo);
        pcl::io::savePCDFileBinary("/home/goa-jh/Dropbox/"+s_algo+"_map.pcd",map);
      }
    }
    else
      ROS_INFO("not successful");
#endif

    return true;
  }


  ros::NodeHandle n_;
  ros::Time stamp_;


protected:

  /**
   * publishing evaluation results
   */
  void publish_results()
  {
    ros::ServiceClient client = n_.serviceClient< ::cob_3d_registration::EvaluationResult>("evaluate");
    ::cob_3d_registration::EvaluationResult srv;

    srv.request.duration = duration_;
    srv.request.memory = memory_usage_kb_;
    srv.request.state = registration_result_;

    Eigen::Matrix4f T = reg_->getTransformation(), T2=Eigen::Matrix4f::Identity();
    //rotate around 180° around z
    Eigen::Vector3f Z;
    Z(0)=Z(1)=0.f;Z(2)=1.f;
    Eigen::AngleAxisf aa(M_PI,Z);
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        T2(i,j)=aa.toRotationMatrix()(i,j);
    T=T2*T;

    for(int i=0; i<4; i++)
      for(int j=0; j<4; j++)
        srv.request.transformation.push_back( T(i,j) );

    if (client.call(srv))
    {
      ROS_DEBUG("published paramters successfully");
    }
    else
    {
      ROS_ERROR("Failed to call paramterlist service");
    }
  }

  /**
   * preprocessing data
   *
   */
  bool do_preprocess(pcl::PointCloud<Point> &pc, cv_bridge::CvImagePtr &cv_ptr, cv::Mat &img_depth) {

#if NEED_GAZEBO_BUG_
    //gazebo bug escaping hack
    for(int i=0; i<pc.size(); i++) {
      if(pc.points[i].z>5||pc.points[i].z<0.5) {
        pc.points.erase(pc.points.begin()+i);
        --i;
      }
    }
    pc.width=pc.size();
    pc.height=1;
#endif

    pcl::PassThrough<Point> pass;
    pass.setInputCloud (pc.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 6.0);
    pass.filter (pc);

    int simulate_distortion;
    if(parameters_.getParam("simulate_distortion",simulate_distortion)&&simulate_distortion) {
      ROS_INFO("simulationg distortion");

      preprocessing::KinectErrorGenerator<Point> filter;
      filter.setInputCloud(pc.makeShared());
      filter.filter(pc);
    }

    double voxelsize;
    if(parameters_.getParam("voxelsize",voxelsize) && voxelsize>0.f) {
      ROS_INFO("voxelize with %f", voxelsize);

      pcl::VoxelGrid<Point> voxel;
      voxel.setInputCloud(pc.makeShared());
      voxel.setLeafSize(voxelsize,voxelsize,voxelsize);
      voxel.filter(pc);

      ROS_INFO("resulting pc with %d points", (int)pc.size());

      if(((cob_3d_registration::Registration_ICP<Point>*)reg_)->getMap()) {
        voxel.setInputCloud(((cob_3d_registration::Registration_ICP<Point>*)reg_)->getMap());
        voxel.setLeafSize(voxelsize,voxelsize,voxelsize);
        voxel.filter(*reg_->getMap());
      }

    }

    return true;
  }

  /**
   * registration process
   *    - setting input data
   *    - measurement
   *    - visualization helper (for debugging)
   */
  bool do_register(const pcl::PointCloud<Point> &pc, const cv_bridge::CvImagePtr &cv_ptr, cv::Mat *img_depth) {
    if(reg_==NULL)
      return false;

    if(cv_ptr) reg_->setInputImage(boost::shared_ptr<const cv::Mat>(new cv::Mat(cv_ptr->image)));
    if(img_depth) reg_->setInputDepthImage(boost::shared_ptr<const cv::Mat>(new cv::Mat(*img_depth)));
    reg_->setInputCloud(pc.makeShared());

    //MemoryOperator mo;
    //boost::thread workerThread(&MemoryOperator::run, &mo);

    //measurement_tools::PrecisionStopWatchAll psw2;
    //measurement_tools::PrecisionStopWatchThread psw;
    bool ret = reg_->compute();
    //duration_ = psw.precisionStop();
    registration_result_ = ret;
    /*{
      std::string s_algo;
      if(parameters_.getParam("algo",s_algo) && s_algo=="rgbdslam")
        duration_ = psw2.precisionStop();
    }*/

    //mo.halt();
    //workerThread.join();

    //memory_usage_kb_ = mo.getKB();

    //ROS_ERROR("time %f", duration_);
    //ROS_ERROR("memory %d", memory_usage_kb_);

    //std::cout<<reg_->getTransformation()<<"\n";

    return ret;
  }

  void
  cameraInfoSubCallback(sensor_msgs::CameraInfo::ConstPtr ci)
  {
    std::string s_algo;
    if(parameters_.getParam("algo",s_algo) && s_algo=="info")

      ((cob_3d_registration::Registration_Infobased<Point>*)reg_)->setKinectParameters(
          ci->P[0],
          ci->P[2],
          ci->P[6]
      );
  }

  ros::ServiceServer reset_server_;
  ros::Subscriber camera_info_sub_;             //subscriber for input pc
  ros::Publisher point_cloud_pub_aligned_;      //publisher for aligned pc
  ros::ServiceServer keyframe_trigger_server_;
  ros::Subscriber point_cloud_sub_;             /// subscriber to input data
  ros::ServiceServer register_ser_;             /// service for evaluation of registration
  tf::TransformBroadcaster tf_br_;
  tf::TransformListener tf_listener_;
  ros::Publisher point_cloud_pub_;              /// publisher for map
  ros::Publisher marker2_pub_;                  /// publish markers for visualization as pc
  ros::Publisher marker_pub_;                   /// publish markers for visualization
  boost::shared_ptr<actionlib::SimpleActionServer<cob_3d_mapping_msgs::TriggerAction> > as_;
  unsigned int ctr_;

  /// parameter bag (containing max, min, step...)
  ParameterBucket parameters_;

  /// registration algorithm
  cob_3d_registration::GeneralRegistration<Point> *reg_;

  bool is_running_;

  //evaluation values

  /// evaluation: duration in seconds
  double duration_;
  /// evaluation: incremental memory usage in kB
  int memory_usage_kb_;
  /// evaluation: success -> true
  bool registration_result_;

  std::string corrected_id_, world_id_, pc_frame_id_;

  enum {E_ALGO_ICP=1,E_ALGO_ICP_LM=2,E_ALGO_GICP=3,E_ALGO_ICP_MOMENTS=4,E_ALGO_ICP_FPFH=5, E_ALGO_ICP_NARF=6, E_ALGO_FASTSLAM=7, E_ALGO_ICP_EDGES=8, E_ALGO_INFO=10, E_ALGO_COR=11, E_ALGO_NONE=0};

  //settings
  void setSettings_ICP(cob_3d_registration::Registration_ICP<Point> *pr) {
    double d=0.;
    int i=0;

    if(parameters_.getParam("corr_dist",d))
      pr->setCorrDist(d);
    if(parameters_.getParam("max_iterations",i))
      pr->setMaxIterations(i);
    if(parameters_.getParam("trf_epsilon",d))
      pr->setTrfEpsilon(pow(10,-d));
    if(parameters_.getParam("outlier_rejection_threshold",d))
      pr->setOutlierRejectionThreshold(d);
    if(parameters_.getParam("use_only_last_refrence",i))
      pr->setUseOnlyLastReference(i!=0);
  }

#ifndef GICP_ENABLE
  void setSettings_ICP_Moments(cob_3d_registration::Registration_ICP_Moments<Point> *pr) {
    setSettings_ICP(pr);

    double d=0.;

    if(parameters_.getParam("moments_radius",d))
      pr->setMomentRadius(d);
  }
  void setSettings_ICP_FPFH(cob_3d_registration::Registration_ICP_FPFH<Point> *pr) {
    setSettings_ICP(pr);

    double d=0.;

    if(parameters_.getParam("fpfh_radius",d))
      pr->setFPFHRadius(d);
  }
  void setSettings_ICP_NARF(cob_3d_registration::Registration_ICP_NARF<Point> *pr) {
    setSettings_ICP(pr);
  }
  void setSettings_ICP_Edges(cob_3d_registration::Registration_ICP_Edges<Point> *pr) {
    double d=0.;

    if(parameters_.getParam("edge_radius",d))
      pr->setRadius(d);
    if(parameters_.getParam("threshold",d))
      pr->setThreshold(d);
    if(parameters_.getParam("distance_threshold",d))
      pr->setDisThreshold(d);
    setSettings_ICP(pr);
  }
  void setSettings_Info(cob_3d_registration::Registration_Infobased<Point> *pr) {
    int i=0;
    double f=0.;

    //if(parameters_.getParam("use_icp",i))
    //  pr->setUseICP(i!=0);

    if(parameters_.getParam("threshold_diff",f))
      pr->setThresholdDiff(f);
    if(parameters_.getParam("threshold_step",f))
      ((cob_3d_registration::Registration_Infobased<Point>*)reg_)->setThresholdStep(f);
    if(parameters_.getParam("min_info",i))
      ((cob_3d_registration::Registration_Infobased<Point>*)reg_)->setMinInfo(i);
    if(parameters_.getParam("max_info",i))
      ((cob_3d_registration::Registration_Infobased<Point>*)reg_)->setMaxInfo(i);
    if(parameters_.getParam("always_relevant_changes",i))
      ((cob_3d_registration::Registration_Infobased<Point>*)reg_)->SetAlwaysRelevantChanges(i!=0);
    if(parameters_.getParam("check_samples",i))
      ((cob_3d_registration::Registration_Infobased<Point>*)reg_)->setCheckBySamples(i!=0);
  }
#endif

  /******************VISUALIZATION******************/
  void publishMarkerPoint(const Point &p, int id, float r, float g, float b, float Size=0.02)
  {
    if(!marker_pub_.getNumSubscribers())
      return;

    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration(0);
    marker.header.frame_id="/head_cam3d_frame";
    //marker.header.stamp = stamp;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = Size;
    marker.scale.y = Size;
    marker.scale.z = Size;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.id = id;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;

    marker_pub_.publish(marker);
  }

  void publishLineMarker(Eigen::Vector3f a, Eigen::Vector3f b, const int id=rand()%111111)
  {
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(id<0?0:4);
    marker.header.frame_id="/head_cam3d_frame";

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1.0;

    marker.id = id;

    marker.points.resize(2);

    marker.points[0].x = a(0);
    marker.points[0].y = a(1);
    marker.points[0].z = a(2);

    marker.points[1].x = b(0);
    marker.points[1].y = b(1);
    marker.points[1].z = b(2);

    marker_pub_.publish(marker);
  }

};

/*
int main(int argc, char **argv) {
#if SHOW_MAP
  setVerbosityLevel(pcl::console::L_DEBUG);
#endif

  ros::init(argc, argv, "registration");

  if(argc>1&&strcmp(argv[1],"eval")==0) { //we're sending paramters...
    RegistrationNodelet tn(true);

    tn.createParamters(argv[2]);
    tn.publishParameters();
    return 0;
  }

  RegistrationNodelet tn;

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}*/

PLUGINLIB_DECLARE_CLASS(cob_3d_registration, RegistrationNodelet, RegistrationNodelet, nodelet::Nodelet)

