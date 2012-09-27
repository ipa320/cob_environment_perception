/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: registration
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-jh
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Nov 9, 2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <gazebo/GetModelState.h>
#include <iostream>

class Capture
{
private:
  double walk_vel, yaw_rate, nick_rate;
  geometry_msgs::Twist cmd;
  trajectory_msgs::JointTrajectory cmd_nick_;

  ros::NodeHandle n_;
  ros::Publisher vel_pub_, nick_pub_;
  ros::Subscriber odo_sub_, point_cloud_sub_, image_sub_, image_depth_sub_;
  tf::TransformListener listener;

  bool moving_, freeze_, first_;
  int not_moving_;
  int phase1_len_, phase2_len_, phase3_len_;

  Eigen::Vector3f absolute_pos_, start_pos_;
  Eigen::Quaternionf absolute_rot_, start_rot_;

  sensor_msgs::PointCloud2ConstPtr last_pc_;
  sensor_msgs::ImageConstPtr last_img_, last_depth_img_;

  std::string prefix_;
  int frame_number_;
  FILE *fp_xml_;
  int mode_; //1 for gazebo, 2 for manual

  bool states_[4];

public:
  void init()
  {
    moving_ = true; //be superspecious
    freeze_ = false;
    first_  = true;

    frame_number_=0;

    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    cmd_nick_.joint_names.push_back("torso_lower_neck_tilt_joint");
    cmd_nick_.joint_names.push_back("torso_pan_joint");
    cmd_nick_.joint_names.push_back("torso_upper_neck_tilt_joint");

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.);
    point.positions.push_back(0.);
    point.positions.push_back(0.);
    point.time_from_start = ros::Duration(0.01);

    cmd_nick_.points.push_back(point);

    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
    nick_pub_= n_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
    odo_sub_ = n_.subscribe("/base_controller/odometry",10,&Capture::odometrySubCallback, this);
    point_cloud_sub_ = n_.subscribe("/cam3d/rgb/points", 1, &Capture::pointCloudSubCallback, this);
    image_sub_ = n_.subscribe("/cam3d/rgb/image_raw", 1, &Capture::imageSubCallback, this);
    image_depth_sub_ = n_.subscribe("/cam3d/depth/image_raw", 1, &Capture::imageSubCallback_depth, this);

    ros::NodeHandle n_private("~");

    n_private.param("mode", mode_, 1);

    n_private.param("walk_vel", walk_vel, 0.01);
    n_private.param("yaw_rate", yaw_rate, 0.01);
    n_private.param("nick_rate", nick_rate, 0.01);
    n_private.param("prefix", prefix_, std::string(""));
    n_private.param("phase1_len", phase1_len_, 3);
    n_private.param("phase2_len", phase2_len_, 10);
    n_private.param("phase3_len", phase3_len_, 3);

    n_private.param("state_trans_x", states_[0], false);
    n_private.param("state_trans_y", states_[1], false);
    n_private.param("state_rot_z", states_[2], false);
    n_private.param("state_rot_x", states_[3], false);

  }

  ~Capture()   { }

  void reset_world() {system("rosservice call /gazebo/reset_world");}

  void run();

  void record_start() {
    char fn[256];
    sprintf(fn, "%sreplay.xml",prefix_.c_str());

    ROS_INFO("creating %s...",fn);
    fp_xml_ = fopen(fn,"w");

    ROS_ASSERT(fp_xml_);

    fputs("<?xml version=\"1.0\" ?>\n", fp_xml_);
    fputs("<replay>\n", fp_xml_);
  }

  void record_end() {
    fputs("</replay>\n", fp_xml_);
    fclose(fp_xml_);
  }

  void record_entry() {
    if(!last_pc_ || !last_img_)
      return;

    freeze_=true;

    if(first_) {
      first_=false;
      start_pos_= absolute_pos_;
      start_rot_= absolute_rot_;
    }

    char fn_pcd[256], fn_img[256], fn_img_depth[256];
    sprintf(fn_pcd, "%spcd_%d.pcd",prefix_.c_str(),frame_number_);
    sprintf(fn_img, "%simg_%d.img",prefix_.c_str(),frame_number_);
    sprintf(fn_img_depth, "%simg_depth_%d.img",prefix_.c_str(),frame_number_);

    frame_number_++;

    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::fromROSMsg(*last_pc_,pc);
    pcl::io::savePCDFileASCII (fn_pcd, pc);

    #ifdef PCL_VERSION_COMPARE
      std::ofstream stream(fn_img);
	  stream << *last_img_;
	  stream.close();
	#else
      //serialize image
      FILE *fp = fopen(fn_img,"wb");
      if(fp)
      {
        uint32_t len = last_img_->serializationLength();

        uint8_t *wptr = new uint8_t[len];
        last_img_->serialize(wptr,0);
        fwrite(wptr, 1, len, fp);
        delete [] wptr;

        fclose(fp);
      }
	#endif

    /*fp = fopen(fn_img_depth,"wb");
    if(fp)
    {
      uint32_t len = last_depth_img_->serializationLength();

      uint8_t *wptr = new uint8_t[len];
      last_depth_img_->serialize(wptr,0);
      fwrite(wptr, 1, len, fp);
      delete [] wptr;

      fclose(fp);
    }*/

    Eigen::Vector3f pos= absolute_pos_ - start_pos_;
    Eigen::Quaternionf rot = absolute_rot_*start_rot_.inverse();

    /*std::cout<<"saving pos:\n"<<pos<<"\n";
    std::cout<<"saving rot:\n"<<rot.toRotationMatrix()<<"\n";
    Eigen::Vector3f v;
    v(0)=0;
    v(1)=0;
    v(2)=1;
    std::cout<<"rotating:\n"<<rot.toRotationMatrix()*v<<"\n";*/

    fputs("\t<frame>\n", fp_xml_);

    fputs("\t\t<pose>\n", fp_xml_);

    fputs("\t\t\t<position>", fp_xml_);
    fprintf(fp_xml_,"%f,%f,%f",pos(0),pos(1),pos(2));
    fputs("</position>\n", fp_xml_);

    fputs("\t\t\t<orientation>", fp_xml_);
    fprintf(fp_xml_,"%f,%f,%f,%f",rot.x(), rot.y(), rot.z(), rot.w());
    fputs("</orientation>\n", fp_xml_);

    fputs("\t\t</pose>\n", fp_xml_);

    fputs("\t\t<pcd>", fp_xml_);
    fputs(fn_pcd, fp_xml_);
    fputs("</pcd>\n", fp_xml_);

    fputs("\t\t<img>", fp_xml_);
    fputs(fn_img, fp_xml_);
    fputs("</img>\n", fp_xml_);

    /*fputs("\t\t<img_depth>", fp_xml_);
    fputs(fn_img_depth, fp_xml_);
    fputs("</img_depth>\n", fp_xml_);*/

    fputs("\t</frame>\n", fp_xml_);

    last_pc_.reset();
    last_img_.reset();

    freeze_=false;
  }

  void
  pointCloudSubCallback(const sensor_msgs::PointCloud2ConstPtr& pc_in)
  {
    if(freeze_)
      return;

    last_pc_ = pc_in;
  }

  void
  imageSubCallback(const sensor_msgs::ImageConstPtr& img_in)
  {
    if(freeze_)
      return;

    last_img_ = img_in;
  }

  void
  imageSubCallback_depth(const sensor_msgs::ImageConstPtr& img_in)
  {
    if(freeze_)
      return;

    last_depth_img_ = img_in;
  }

  void
  odometrySubCallback(const nav_msgs::Odometry &odometry)
  {
    if(freeze_ || mode_==2)
      return;

    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/base_footprint", "/head_cam3d_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      moving_ = true;
      return;
    }

    ros::ServiceClient client = n_.serviceClient<gazebo::GetModelState>("/gazebo/get_model_state");
    gazebo::GetModelState srv;

    srv.request.model_name = "robot";

    if(!client.call(srv)) {
      ROS_ERROR("service call for gazebo model state failed");
      moving_ = true;
      return;
    }

    Eigen::Vector3f pos_base, pos_rel;
    Eigen::Quaternionf rot_base, rot_rel;

    /*pos_base(0) = odometry.pose.pose.position.x;
    pos_base(1) = odometry.pose.pose.position.y;
    pos_base(2) = odometry.pose.pose.position.z;*/
    pos_base(0) = srv.response.pose.position.x;
    pos_base(1) = srv.response.pose.position.y;
    pos_base(2) = srv.response.pose.position.z;

    pos_rel(0) = transform.getOrigin().m_floats[0];
    pos_rel(1) = transform.getOrigin().m_floats[1];
    pos_rel(2) = transform.getOrigin().m_floats[2];

    /*rot_base.x() = odometry.pose.pose.orientation.x;
    rot_base.y() = odometry.pose.pose.orientation.y;
    rot_base.z() = odometry.pose.pose.orientation.z;
    rot_base.w() = odometry.pose.pose.orientation.w;*/
    rot_base.x() = srv.response.pose.orientation.x;
    rot_base.y() = srv.response.pose.orientation.y;
    rot_base.z() = srv.response.pose.orientation.z;
    rot_base.w() = srv.response.pose.orientation.w;

    rot_rel.x() = transform.getRotation().getX();
    rot_rel.y() = transform.getRotation().getY();
    rot_rel.z() = transform.getRotation().getZ();
    rot_rel.w() = transform.getRotation().getW();

    //rematch
    {
      Eigen::Vector3f Y;
      Y(0)=Y(2)=0;Y(1)=1;
      Eigen::Vector3f X;
      X(1)=X(2)=0;X(0)=1;
      Eigen::AngleAxisf r1(-M_PI/2,Y), r2(-M_PI/2,X);

      Eigen::Matrix3f R=r1.toRotationMatrix()*r2.toRotationMatrix();

      /*Eigen::Vector3f Z;
      Z(1)=Z(0)=0;Z(2)=1;

      std::cout<<"X\n"<<(R*X)<<"\n";
      std::cout<<"Y\n"<<(R*Y)<<"\n";
      std::cout<<"Z\n"<<(R*Z)<<"\n";*/

      rot_base = R*rot_base;
      pos_base = R*pos_base;
    }

    //std::cout<<"rel: \n"<<pos_rel<<"\n";
    //std::cout<<"base: \n"<<pos_base<<"\n";

    Eigen::Vector3f pos= pos_base+pos_rel;
    Eigen::Quaternionf rot = rot_base*rot_rel;

    //ROS_INFO("rotated %f°",180/M_PI*rot.angularDistance(Eigen::Quaternionf(Eigen::AngleAxisf(0,Eigen::Vector3f(0,1,0)))));

    moving_ =
        (pos-absolute_pos_).norm()>0.00001 ||
        rot.angularDistance(absolute_rot_)>0.00001;

    absolute_pos_= pos;
    absolute_rot_= rot;

    if(!moving_) {
      not_moving_++;
    }
    else
      not_moving_=0;

  }

  void buildState() {
    if(mode_==2) {
      ROS_INFO("please move camera to desired position");
      ROS_INFO("now enter:");

      float x=0,y=0,a=0;

      std::cout<<"translation x (in cm): ";
      std::cin>>x;
      std::cout<<"translation y (in cm): ";
      std::cin>>y;
      std::cout<<"rotation (degree): ";
      std::cin>>a;

      x/=100.f;
      y/=100.f;
      a*=M_PI/180.f;

      absolute_pos_(0)=x;
      absolute_pos_(1)=0;
      absolute_pos_(2)=y;

      Eigen::Vector3f Y;
      Y(0)=Y(2)=0.f;Y(1)=1.f;
      Eigen::AngleAxisf aa(a,Y);
      absolute_rot_=Eigen::Quaternionf(aa);
    }
    else
    {
      cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

      if(frame_number_ >= phase1_len_&& frame_number_<phase1_len_+phase2_len_) {
        if(states_[0]) state2_x();
        if(states_[1]) state2_y();
        if(states_[2]) state3();
        if(states_[3]) state4();
      }
    }

  }

  void state2_x() {
    static bool forw=true;
    if(absolute_pos_(0)<-2.5)
      forw=false;
    if(absolute_pos_(0)>-0.5)
      forw=true;

    if(forw)
      cmd.linear.x = - walk_vel;
    else
      cmd.linear.x = walk_vel;
  }

  void state2_y() {
    static bool forw=true;
    if(absolute_pos_(1)<-1)
      forw=false;
    if(absolute_pos_(1)>1)
      forw=true;

    if(forw)
      cmd.linear.y = - walk_vel;
    else
      cmd.linear.y = walk_vel;
  }

  void state3() {
    static bool forw=true;

    static float yaw = 0.;

    if(yaw<-0.4)
      forw=false;
    if(yaw>0.4)
      forw=true;

    if(forw)
      yaw += -nick_rate;
    else
      yaw +=  nick_rate;

    cmd_nick_.points[0].positions[1] = yaw;
  }

  void state4() {
    static bool forw=true;

    static float yaw = 0.;

    if(yaw<-0.4)
      forw=false;
    if(yaw>0.4)
      forw=true;

    if(forw)
      yaw += -nick_rate;
    else
      yaw +=  nick_rate;

    cmd_nick_.points[0].positions[0] = yaw;
    cmd_nick_.points[0].positions[2] = 1.5*yaw;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "capture");

  Capture tpk;
  tpk.init();

  tpk.reset_world();
  boost::thread workerThread(&Capture::run, &tpk);

  ros::spin();

  return(0);
}

void Capture::run()
{
  record_start();

  for(;ros::ok();)
  {

    ROS_INFO("move");

    buildState();

    vel_pub_.publish(cmd);
    nick_pub_.publish(cmd_nick_);

    usleep(1000*200);
    while(moving_&&not_moving_<20) usleep(10000);

    //getchar(); //testing...

    Eigen::Matrix3f t =absolute_rot_.toRotationMatrix();
    float rx = atan2(t(2,1), t(2,2));
    float ry = asin(-t(2,0));
    float rz = atan2(t(1,0), t(0,0));

    /*std::cout<<"origin: \n"<<absolute_pos_<<"\n";
    std::cout<<"rot: "
        <<rx<<" "
        <<ry<<" "
        <<rz<<" "
        <<"\n";*/

    record_entry();

    if(frame_number_ >= phase1_len_+phase2_len_+phase3_len_)
      break;
  }
  ROS_INFO("finished");
  record_end();
  ros::shutdown();
}
