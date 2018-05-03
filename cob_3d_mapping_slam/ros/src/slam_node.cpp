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
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
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
/*
 * slam_node.cpp
 *
 *  Created on: 17.05.2012
 *      Author: josh
 */

#define DEBUG_


// ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//MSGS
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/CurvedPolygon_Array.h>
#include <visualization_msgs/Marker.h>
#include <arm_navigation_msgs/CollisionMap.h>

//DEBUG
#ifdef DEBUG_
#include <cob_3d_mapping_slam/curved_polygons/debug_interface.h>
#endif

//DOF6
#include <cob_3d_mapping_slam/dof/tflink.h>
#include <cob_3d_mapping_slam/dof/dof_uncertainty.h>
#include <cob_3d_mapping_slam/dof/dof_variance.h>

//SLAM
#include <cob_3d_mapping_slam/slam/context.h>
#include <cob_3d_mapping_slam/slam/node.h>
#include <cob_3d_mapping_slam/slam/dummy/robot.h>

//IMPL
#include <cob_3d_mapping_slam/curved_polygons/objctxt.h>
#include <cob_3d_mapping_slam/curved_polygons/key.h>




#define  Pr  .299
#define  Pg  .587
#define  Pb  .114



//  public-domain function by Darel Rex Finley
//
//  The passed-in RGB values can be on any desired scale, such as 0 to
//  to 1, or 0 to 255.  (But use the same scale for all three!)
//
//  The "change" parameter works like this:
//    0.0 creates a black-and-white image.
//    0.5 reduces the color saturation by half.
//    1.0 causes no change.
//    2.0 doubles the color saturation.
//  Note:  A "change" value greater than 1.0 may project your RGB values
//  beyond their normal range, in which case you probably should truncate
//  them to the desired range before trying to use them in an image.

void changeSaturation(float *R, float *G, float *B, float change) {

  float  P=sqrtf(
      (*R)*(*R)*Pr+
      (*G)*(*G)*Pg+
      (*B)*(*B)*Pb ) ;

  *R=std::min(std::max(P+((*R)-P)*change,0.f),1.f);
  *G=std::min(std::max(P+((*G)-P)*change,0.f),1.f);
  *B=std::min(std::max(P+((*B)-P)*change,0.f),1.f); }


class As_Node
{
protected:
  ros::NodeHandle n_;
  tf::TransformListener tf_listener_;
public:
  As_Node(): n_("~") {
  }

  virtual ~As_Node() {}

  virtual void onInit()=0;

  void start() {

  }
};

class As_Nodelet : public  pcl_ros::PCLNodelet
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

template <typename Parent>
class SLAM_Node : public Parent
{
  typedef DOF6::DOF6_Source<DOF6::TFLinkvf,DOF6::DOF6_Uncertainty<Dummy::RobotParametersSlow,float> > DOF6;
  typedef Slam::Node<Slam_CurvedPolygon::OBJCTXT<DOF6> > Node;


  ros::Subscriber curved_poly_sub_;
  ros::Subscriber shapes_sub_;
  ros::Publisher  map_pub_, debug_pub_, debug2_pub_, outline_pub_, collision_map_pub_;

  cob_3d_marker::MarkerContainer marker_cont;
  cob_3d_marker::MarkerPublisher marker_pub;

  std::string world_id_, frame_id_;

  bool use_real_world_color_, use_odometry_;
  bool dbg_show_grid_;

  bool invert_;
  ros::Time start_ts_;

  Slam::Context<Slam_CurvedPolygon::KEY<DOF6>, Node> *ctxt_;

  Eigen::Matrix4f M_last_;
  bool first_;

  cob_3d_mapping_msgs::CurvedPolygon convert(const cob_3d_mapping_msgs::Shape &sh) {
    cob_3d_mapping_msgs::CurvedPolygon r;
    r.ID = sh.id;
    r.weight = sh.weight;

    ROS_ASSERT_MSG(r.parameter.size()==6, "incompatible parameters");
    for(int i=0; i<6; i++)
      r.parameter[i] = sh.params[i];

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(sh.points[0],pc);

    for(size_t i=0; i<pc.size(); i++) {
      cob_3d_mapping_msgs::polyline_point pt;
      pt.x = pc[i].x;
      pt.y = pc[i].y;
      pt.edge_prob = pc[i].z;
      r.polyline.push_back(pt);
    }

    return r;
  }

public:
  // Constructor
  SLAM_Node(): first_(true), invert_(false), use_real_world_color_(true), use_odometry_(false), dbg_show_grid_(true),
  marker_pub(debug2_pub_)
  {
  }

  virtual ~SLAM_Node()
  {}

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);
    //curved_poly_sub_ = this->n_.subscribe("/curved_polygons", 1, &SLAM_Node<Parent>::cbPolygons, this);
    shapes_sub_ = this->n_.subscribe("/shapes_array", 1, &SLAM_Node<Parent>::cbShapes, this);
    map_pub_ = n->advertise<visualization_msgs::Marker>("map", 10);
    debug_pub_ = n->advertise<visualization_msgs::Marker>("debug", 1);
    debug2_pub_ = n->advertise<visualization_msgs::Marker>("debug2", 100);
    outline_pub_ = n->advertise<visualization_msgs::Marker>("outline", 1);
    collision_map_pub_ = n->advertise<arm_navigation_msgs::CollisionMap>("collision_map", 10);

    double tr_dist = 1.f, rot_dist=0.8f;
    double tr_speed = 0.6, rot_speed=0.6;

    n->getParam("translation_size",tr_dist);
    n->getParam("rotation_size",rot_dist);

    n->getParam("translation_speed",tr_speed); //TODO:
    n->getParam("rotation_speed",rot_speed);

    n->getParam("world_id",world_id_);
    n->getParam("frame_id",frame_id_);

    n->getParam("invert",invert_);

    n->getParam("real_world_color",use_real_world_color_);
    n->getParam("use_odometry",use_odometry_);
    n->getParam("dbg_show_grid",dbg_show_grid_);

    ctxt_ = new Slam::Context<Slam_CurvedPolygon::KEY<DOF6>, Node>(tr_dist, rot_dist);
  }

  void
  cbShapes(cob_3d_mapping_msgs::ShapeArray::ConstPtr cpa)
  {
    std::vector< ::std_msgs::ColorRGBA> color;
    cob_3d_mapping_msgs::CurvedPolygon_Array::Ptr ar(new cob_3d_mapping_msgs::CurvedPolygon_Array());
    for(size_t i=0; i<cpa->shapes.size(); i++) {
      if(cpa->shapes[i].weight>500)
      {
        ar->polygons.push_back(convert(cpa->shapes[i]));
        color.push_back(cpa->shapes[i].color);
      }
    }
    ar->header = cpa->header;
    cbPolygons2(ar, color);
  }

  void
  cbPolygons(cob_3d_mapping_msgs::CurvedPolygon_Array::ConstPtr cpa)
  {
    cbPolygons2(cpa,std::vector< ::std_msgs::ColorRGBA>());
  }

  void
  cbPolygons2(cob_3d_mapping_msgs::CurvedPolygon_Array::ConstPtr cpa, const std::vector< ::std_msgs::ColorRGBA> &color)
  {
    if(!ctxt_) return;

    static bool lock=false;

    if(lock) return;

    lock=true;
    double start_time = ros::Time::now().toSec();
    Debug::Interface::get().setTime(ros::Time::now().toSec());

    ctxt_->startFrame(cpa->header.stamp.toSec());

    if(use_odometry_) {
      if(world_id_.size()>0&&frame_id_.size()>0) {
        tf::StampedTransform transform;
        try
        {
          std::stringstream ss2;
          this->tf_listener_.waitForTransform(frame_id_, world_id_, cpa->header.stamp, ros::Duration(0.2));
          this->tf_listener_.lookupTransform(frame_id_, world_id_, cpa->header.stamp, transform);

          Eigen::Affine3d af;
          tf::TransformTFToEigen(transform, af);
          Eigen::Matrix4f _M = af.matrix().cast<float>(), M, R=Eigen::Matrix4f::Identity();

          std::cout<<"TSDIFF "<<(transform.stamp_-cpa->header.stamp).toSec()<<"\n";

          //        Eigen::Vector3f x=Eigen::Vector3f::Zero(), z=Eigen::Vector3f::Zero();
          //        x(0)=1;
          //        z(2)=1;
          //        Eigen::AngleAxisf aa1(-M_PI/2,x);
          //        Eigen::AngleAxisf aa2(M_PI/2,z);
          //
          //        R.topLeftCorner(3,3) = aa2.toRotationMatrix()*aa1.toRotationMatrix();
          //        _M = R*_M;

          if(first_) {
            start_ts_ = cpa->header.stamp;
            first_=false;
            M_last_ = _M;
          }

          M = M_last_.inverse()*_M;
          if(invert_)
            M = M.inverse().eval();

          std::cout<<"ODOMETRY\n"<<M<<"\n";
          std::cout<<"_M_\n"<<_M<<"\n";

          M = M*ctxt_->getPath().getLocal().link_.getTF4();

          ctxt_->getPath().getLocal().link_.setVariance(
              std::max(ctxt_->getPath().getLocal().link_.getSource2()->getTranslationVariance()*0.8f, 0.05f),
              M.col(3).head<3>(),
              ctxt_->getPath().getLocal().link_.getSource2()->getRotationVariance()*0.8f,
              (typename DOF6::TROTATION)(M.topLeftCorner(3,3))
          );
          M_last_ = _M;

          std::cout<<"ODOMETRY LINK\n"<<ctxt_->getPath().getLocal().link_<<"\n";

        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("[slam] : %s",ex.what());
          lock=false;
          return;
        }
      }

      ROS_INFO("timestamp %f", (float)(ros::Time::now()-start_ts_).toSec());
    }

    for(size_t i=0; i<cpa->polygons.size(); i++) {
      Slam_CurvedPolygon::ex_curved_polygon ep = cpa->polygons[i];
      if(i<color.size())
        ep.setColor(color[i]);
      *ctxt_ += ep;
    }
    ctxt_->finishFrame();


    //check path
    {
      Eigen::Matrix3f tmp_rot = Eigen::Matrix3f::Identity();
      Eigen::Matrix3f tmp_rot2 = Eigen::Matrix3f::Identity();
      Eigen::Vector3f tmp_tr  = Eigen::Vector3f::Zero();
      const Slam::SWAY<Node> *n = &ctxt_->getPath().getLocal();
      while(n)
      {

        tmp_tr = tmp_rot2*tmp_tr + n->link_.getTranslation();
        tmp_rot = ((Eigen::Matrix3f)n->link_.getRotation())*tmp_rot;
        tmp_rot2= ((Eigen::Matrix3f)n->link_.getRotation());

        std::cout<<"ROTn\n"<<(::DOF6::EulerAnglesf)n->link_.getRotation()<<"\n";
        std::cout<<"TRn\n"<<n->link_.getTranslation()<<"\n";

        std::cout<<"con\n";
        size_t next_id = n->id_-1;
        const Slam::SWAY<Node> *n2 = NULL;
        for(size_t i=0; i<n->node_->getConnections().size(); i++)
          if(n->node_->getConnections()[i].id_ == next_id) {
            n2 = &n->node_->getConnections()[i];
            break;
          }
        n = n2;
      }
      std::cout<<"ROT1\n"<<(::DOF6::EulerAnglesf)tmp_rot<<"\n";
      std::cout<<"TR1\n"<<tmp_tr<<"\n";
    }



    visualization_msgs::Marker marker_map, marker_outline;
    marker_map.header = cpa->header;
    marker_map.pose.position.x = 0;
    marker_map.pose.position.y = 0;
    marker_map.pose.position.z = 0;
    marker_map.pose.orientation.x = 0.0;
    marker_map.pose.orientation.y = 0.0;
    marker_map.pose.orientation.z = 0.0;
    marker_map.pose.orientation.w = 1.0;
    marker_map.action = visualization_msgs::Marker::ADD;
    marker_map.color.r = marker_map.color.g = marker_map.color.b =  marker_map.color.a = 1;

    marker_map.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker_map.scale.x = marker_map.scale.y = marker_map.scale.z = 1;
    marker_map.id = 5;

    marker_outline = marker_map;
    marker_outline.id = 9;
    marker_outline.type = visualization_msgs::Marker::LINE_LIST;
    marker_outline.scale.x = marker_outline.scale.y = marker_outline.scale.z = 0.01;

    std_msgs::ColorRGBA line_c;
    visualization_msgs::Marker marker_dbg = marker_map;

    Eigen::Matrix3f tmp_rot = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f tmp_rot2 = Eigen::Matrix3f::Identity();
    Eigen::Vector3f tmp_tr  = Eigen::Vector3f::Zero();
    const Slam::SWAY<Node> *n = &ctxt_->getPath().getLocal();
    int node_num=1;
    while(n)
    {
      tmp_tr = tmp_rot2*tmp_tr + n->link_.getTranslation();
      tmp_rot = ((Eigen::Matrix3f)n->link_.getRotation())*tmp_rot;
      tmp_rot2= ((Eigen::Matrix3f)n->link_.getRotation());

      ::std_msgs::ColorRGBA col;
      {
        srand(node_num++);
        int rnd=rand();
        col.a=1;
        col.r=((rnd>>0)&0xff)/255.;
        col.g=((rnd>>8)&0xff)/255.;
        col.b=((rnd>>16)&0xff)/255.;
      }

      marker_cont << cob_3d_marker::MarkerClean();
      marker_cont>>&marker_pub;
      marker_cont.clear();
      marker_cont<<new cob_3d_marker::MarkerList_Line(2)<<new cob_3d_marker::MarkerList_Arrow(3)<<new cob_3d_marker::MarkerList_Text(4)<<new cob_3d_marker::MarkerList_Text(5);

      arm_navigation_msgs::CollisionMap collision_map;

      for(size_t i=0; i<n->node_->getContext().getObjs().size(); i++)
      {

        if(use_real_world_color_) {
          col = n->node_->getContext().getObjs()[i]->getData().getColor();
          changeSaturation(&col.r,&col.g,&col.b, 1.5f);
        }

        geometry_msgs::Point line_p;

        for(size_t j=0; j<n->node_->getContext().getObjs()[i]->getData().getPoints3D().size(); j++) {
          Eigen::Vector3f v = tmp_rot*n->node_->getContext().getObjs()[i]->getData().getPoints3D()[j]+tmp_tr;
          float edge = n->node_->getContext().getObjs()[i]->getData().getOutline()[j](2);

          ::std_msgs::ColorRGBA col_out;
          col_out.a=1;
          col_out.g=0;
          col_out.r=edge;
          col_out.b=1-edge;
          line_p.x = v(0);
          line_p.y = v(1);
          line_p.z = v(2);
          marker_outline.points.push_back(line_p);
          marker_outline.colors.push_back(col_out);

          v = tmp_rot*n->node_->getContext().getObjs()[i]->getData().getPoints3D()[(j+1)%n->node_->getContext().getObjs()[i]->getData().getPoints3D().size()]+tmp_tr;
          edge = n->node_->getContext().getObjs()[i]->getData().getOutline()[(j+1)%n->node_->getContext().getObjs()[i]->getData().getPoints3D().size()](2);

          col_out.r=edge;
          col_out.b=1-edge;
          line_p.x = v(0);
          line_p.y = v(1);
          line_p.z = v(2);
          marker_outline.points.push_back(line_p);
          marker_outline.colors.push_back(col_out);
        }

        std::vector<Eigen::Vector3f> tris;
        n->node_->getContext().getObjs()[i]->getData().getTriangles(tris);

        for(size_t j=0; j<tris.size(); j++)
        {
          Eigen::Vector3f v=tmp_rot*tris[j]+tmp_tr;
          line_p.x = v(0);
          line_p.y = v(1);
          line_p.z = v(2);
          marker_map.points.push_back(line_p);
          marker_map.colors.push_back(col);
        }

        if(dbg_show_grid_ && debug_pub_.getNumSubscribers()>0) {
          std::vector<std::vector<Eigen::Vector3f> > pts;
          n->node_->getContext().getObjs()[i]->getData().getControlPoints(pts);
          for(size_t j=0; j<pts.size(); j++)
          {
            for(size_t k=0; k<pts[j].size()-1; k++)
            {
              Eigen::Vector3f v=tmp_rot*pts[j][k]+tmp_tr;
              line_p.x = v(0);
              line_p.y = v(1);
              line_p.z = v(2);
              line_c=col;
              marker_dbg.points.push_back(line_p);
              marker_dbg.colors.push_back(line_c);

              v=tmp_rot*pts[j][k+1]+tmp_tr;
              line_p.x = v(0);
              line_p.y = v(1);
              line_p.z = v(2);
              marker_dbg.points.push_back(line_p);
              marker_dbg.colors.push_back(line_c);

              if(j+1!=pts.size()) {
                v=tmp_rot*pts[j][k]+tmp_tr;
                line_p.x = v(0);
                line_p.y = v(1);
                line_p.z = v(2);
                line_c=col;
                marker_dbg.points.push_back(line_p);
                marker_dbg.colors.push_back(line_c);
                v=tmp_rot*pts[j+1][k]+tmp_tr;
                line_p.x = v(0);
                line_p.y = v(1);
                line_p.z = v(2);
                line_c=col;
                marker_dbg.points.push_back(line_p);
                marker_dbg.colors.push_back(line_c);
              }

            }
          }
        }


        if(debug2_pub_.getNumSubscribers()>0) {
          marker_cont<<n->node_->getContext().getObjs()[i]->getData().getSurface();

          char buffer[128];
          static int nth=0;
          sprintf(buffer, "%d",nth++);
          n->node_->getContext().getObjs()[i]->getData().getOutline().debug_svg(buffer);

          ((cob_3d_marker::MarkerList_Text*)marker_cont.get(5).get())->addText(n->node_->getContext().getObjs()[i]->getData().getNearestPoint(), buffer, 0.05f);

        }

        //BB
        {
          ::std_msgs::ColorRGBA col;
          col.g = col.r = col.b=0;
          if(n->node_->getContext().getObjs()[i]->getProcessed())
            col.g = 1.f;
          else
            col.b = 1.f;

          addBB(marker_dbg, n->node_->getContext().getObjs()[i]->getData().getBB(), col, tmp_rot,tmp_tr);
        }

        {
        arm_navigation_msgs::OrientedBoundingBox box;

        box.center.x = n->node_->getContext().getObjs()[i]->getData().getBB().getCenter()(0);
        box.center.y = n->node_->getContext().getObjs()[i]->getData().getBB().getCenter()(1);
        box.center.z = n->node_->getContext().getObjs()[i]->getData().getBB().getCenter()(2);

        box.extents.x = n->node_->getContext().getObjs()[i]->getData().getBB().getExtension()(0);
        box.extents.y = n->node_->getContext().getObjs()[i]->getData().getBB().getExtension()(1);
        box.extents.z = n->node_->getContext().getObjs()[i]->getData().getBB().getExtension()(2);

        Eigen::AngleAxisf aa(n->node_->getContext().getObjs()[i]->getData().getBB().getAxis());
        box.axis.x = aa.axis()(0);
        box.axis.y = aa.axis()(1);
        box.axis.z = aa.axis()(2);
        box.angle = aa.angle();

        collision_map.boxes.push_back(box);
        }
      }
      marker_cont>>&marker_pub;

      collision_map.header = cpa->header;
      //TODO: check frame_id
      if(collision_map_pub_.getNumSubscribers()>0)
        collision_map_pub_.publish(collision_map);

      //BB of node
      //::std_msgs::ColorRGBA col;
      col.g = col.r = col.b=0;
      col.g = 1.f;
      col.b = 1.f;

      if(node_num==2)
      {
        col.g = col.r = col.b=0;
        col.r = 1.f;
      }

      addBB(marker_dbg, n->node_->getContext().getBoundingBox(), col, tmp_rot,tmp_tr);

      size_t next_id = n->id_-1;
      const Slam::SWAY<Node> *n2 = NULL;
      for(size_t i=0; i<n->node_->getConnections().size(); i++)
        if(n->node_->getConnections()[i].id_ == next_id) {
          n2 = &n->node_->getConnections()[i];
          break;
        }
      n = n2;
      //break;
    }

    std::cout<<"TF AFTER ALL\n"<<tmp_rot<<"\n"<<tmp_tr<<std::endl;

    map_pub_.publish(marker_map);
    outline_pub_.publish(marker_outline);

    if(debug_pub_.getNumSubscribers()>0) {
      marker_dbg.id = 3;
      marker_dbg.type = visualization_msgs::Marker::LINE_LIST;
      marker_dbg.scale.x = marker_dbg.scale.y = marker_dbg.scale.z = 0.005;


      geometry_msgs::Point line_p;
      pcl::PointXYZRGB p;
      Eigen::Vector3f from,to,temp;
      while(Debug::Interface::get().getArrow(from,to,p.r,p.g,p.b))
      {
        line_p.x = from(0);
        line_p.y = from(1);
        line_p.z = from(2);
        line_c.r=p.r/255.f;
        line_c.g=p.g/255.f;
        line_c.b=p.b/255.f;
        line_c.a=1;
        //p.r>100?marker_dbg.points.push_back(line_p):marker_cor2.points.push_back(line_p);
        marker_dbg.points.push_back(line_p);
        marker_dbg.colors.push_back(line_c);
        line_p.x = to(0);
        line_p.y = to(1);
        line_p.z = to(2);
        //p.r>100?marker_dbg.points.push_back(line_p):marker_cor2.points.push_back(line_p);
        marker_dbg.points.push_back(line_p);
        marker_dbg.colors.push_back(line_c);
      }

      //FoV
      {
        line_c.r=line_c.g=1;
        line_c.b=0;

        Eigen::Vector3f mi,ma, ed[8];
        mi(0)=-0.43f;
        mi(1)=-0.36f;
        ma(0)=0.43f;
        ma(1)=0.36f;
        mi(2)=ma(2)=1.f;

        for(int i=0; i<4; i++)
          ed[i]=mi;

        ed[1](0)=ma(0);
        ed[2](0)=ma(0);
        ed[2](1)=ma(1);
        ed[3](1)=ma(1);

        for(int i=0; i<4; i++) {
          ed[i+4]=ed[i];
          ed[i]*=0.4f;
          ed[i+4]*=8;}

        Eigen::Vector3f t;
        t(0)=t(1)=0;
        t(2)=0.4f;
        for(int i=0; i<8; i++) ed[i] -= t;

        for(int i=0; i<4; i++) {
          line_p.x = ed[i](0);
          line_p.y = ed[i](1);
          line_p.z = ed[i](2);
          marker_dbg.points.push_back(line_p);
          marker_dbg.colors.push_back(line_c);
          line_p.x = ed[(i+1)%4](0);
          line_p.y = ed[(i+1)%4](1);
          line_p.z = ed[(i+1)%4](2);
          marker_dbg.points.push_back(line_p);
          marker_dbg.colors.push_back(line_c);
        }

        for(int i=0; i<4; i++) {
          line_p.x = ed[i+4](0);
          line_p.y = ed[i+4](1);
          line_p.z = ed[i+4](2);
          marker_dbg.points.push_back(line_p);
          marker_dbg.colors.push_back(line_c);
          line_p.x = ed[(i+1)%4+4](0);
          line_p.y = ed[(i+1)%4+4](1);
          line_p.z = ed[(i+1)%4+4](2);
          marker_dbg.points.push_back(line_p);
          marker_dbg.colors.push_back(line_c);
        }

        for(int i=0; i<4; i++) {
          line_p.x = ed[i](0);
          line_p.y = ed[i](1);
          line_p.z = ed[i](2);
          marker_dbg.points.push_back(line_p);
          marker_dbg.colors.push_back(line_c);
          line_p.x = ed[i+4](0);
          line_p.y = ed[i+4](1);
          line_p.z = ed[i+4](2);
          marker_dbg.points.push_back(line_p);
          marker_dbg.colors.push_back(line_c);
        }

      }

      debug_pub_.publish(marker_dbg);
    }

    lock = false;

    ROS_ERROR("took %f", ros::Time::now().toSec()-start_time);

  }
private:

  template<typename T>
  void addBB(visualization_msgs::Marker &marker_dbg, const T&bb, const ::std_msgs::ColorRGBA &col, const Eigen::Matrix3f &tmp_rot, const Eigen::Vector3f &tmp_tr) {
    geometry_msgs::Point line_p;
    Eigen::Vector3f edges[8];

    bb.get8Edges(edges);
    for(int i=0; i<8; i++)
      edges[i] = tmp_rot*edges[i]+tmp_tr;

    for(int i=0; i<4; i++)
    {
      line_p.x = edges[2*i](0);
      line_p.y = edges[2*i](1);
      line_p.z = edges[2*i](2);
      marker_dbg.points.push_back(line_p);
      marker_dbg.colors.push_back(col);
      line_p.x = edges[2*i+1](0);
      line_p.y = edges[2*i+1](1);
      line_p.z = edges[2*i+1](2);
      marker_dbg.points.push_back(line_p);
      marker_dbg.colors.push_back(col);
    }

    int conv[]={0,1,4,5};
    for(int j=0; j<4; j++)
    {
      int i=conv[j];
      line_p.x = edges[i](0);
      line_p.y = edges[i](1);
      line_p.z = edges[i](2);
      marker_dbg.points.push_back(line_p);
      marker_dbg.colors.push_back(col);
      line_p.x = edges[i+2](0);
      line_p.y = edges[i+2](1);
      line_p.z = edges[i+2](2);
      marker_dbg.points.push_back(line_p);
      marker_dbg.colors.push_back(col);
    }

    for(int i=0; i<4; i++)
    {
      line_p.x = edges[i](0);
      line_p.y = edges[i](1);
      line_p.z = edges[i](2);
      marker_dbg.points.push_back(line_p);
      marker_dbg.colors.push_back(col);
      line_p.x = edges[i+4](0);
      line_p.y = edges[i+4](1);
      line_p.z = edges[i+4](2);
      marker_dbg.points.push_back(line_p);
      marker_dbg.colors.push_back(col);
    }
  }
};

#ifdef COMPILE_NODELET

typedef SLAM_Node<pcl::PointXYZ,pcl::PointXYZRGB,As_Nodelet> _SLAM_Nodelet;

PLUGINLIB_EXPORT_CLASS(_SLAM_Nodelet, nodelet::Nodelet);

#else

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam");

  SLAM_Node<As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
