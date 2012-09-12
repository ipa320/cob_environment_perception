/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Waqas Tanveer, email:Waqas.Tanveer@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 04/2012
 * ToDo:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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

//##################
//#### includes ####


#include <cob_3d_visualization/shape_marker.h>
#include <cob_3d_visualization/shape_visualization.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
//#include <Eigen/Core>
//#include <cob_3d_mapping_geometry_map/geometry_map.h>
#define PI 3.14159265

using namespace cob_3d_mapping;

std::vector<int> movedShapesIndexes;
std::vector<int> interactedShapes ;

// Global variables needed for ModifyMap service
cob_3d_mapping_msgs::ModifyMap::Request req ;
cob_3d_mapping_msgs::ModifyMap::Response res;
Eigen::Quaternionf quatInit ;
Eigen::Vector3f oldCentroid ;
Eigen::Matrix4f transInit;
Eigen::Affine3f affineInit;
Eigen::Matrix4f transInitInv;


void ShapeMarker::getShape (cob_3d_mapping_msgs::Shape& shape) {
  shape_ = shape ;
}
void ShapeVisualization::setShapePosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)//,const cob_3d_mapping_msgs::Shape& shape)
{

  cob_3d_mapping_msgs::ShapeArray map_msg;
  map_msg.header.frame_id="/map";
  map_msg.header.stamp = ros::Time::now();

  int shape_id;
//  Eigen::Affine3f trans;
  stringstream name(feedback->marker_name);
  Eigen::Quaternionf quat;

  Eigen::Matrix3f rotationMat;
  Eigen::MatrixXf rotationMatInit;

  Eigen::Vector3f vec;
  Eigen::Vector3f vecNew;
  Eigen::Vector3f newCentroid;
  Eigen::Matrix4f transSecondStep;
//  Eigen::Affine3f transformInit ;


  if (feedback->marker_name != "Text"){
    name >> shape_id ;

    cob_3d_mapping::Polygon p;
    cob_3d_mapping::fromROSMsg (sha.shapes.at(shape_id), p);

    if (feedback->event_type == 2 && feedback->menu_entry_id == 5){
      quatInit.x() = (float)feedback->pose.orientation.x ;           //normalized
      quatInit.y() = (float)feedback->pose.orientation.y ;
      quatInit.z() = (float)feedback->pose.orientation.z ;
      quatInit.w() = (float)feedback->pose.orientation.w ;

      oldCentroid (0) = (float)feedback->pose.position.x ;
      oldCentroid (1) = (float)feedback->pose.position.y ;
      oldCentroid (2) = (float)feedback->pose.position.z ;

      quatInit.normalize() ;

      rotationMatInit = quatInit.toRotationMatrix() ;

      transInit.block(0,0,3,3) << rotationMatInit ;
      transInit.col(3).head(3) << oldCentroid(0) , oldCentroid(1), oldCentroid(2) ;
      transInit.row(3) << 0,0,0,1 ;

//      Eigen::Matrix4f transInitInverse(transInit.inverse()) ;
      transInitInv = transInit.inverse() ;
      Eigen::Affine3f affineInitFinal (transInitInv) ;
      affineInit = affineInitFinal ;

      std::cout << "transInit : " << "\n"    << affineInitFinal.matrix() << "\n" ;
    }

    if (feedback->event_type == 5){

      quat.x() = (float)feedback->pose.orientation.x ;           //normalized
      quat.y() = (float)feedback->pose.orientation.y ;
      quat.z() = (float)feedback->pose.orientation.z ;
      quat.w() = (float)feedback->pose.orientation.w ;

      quat.normalize() ;

      rotationMat = quat.toRotationMatrix() ;

      vec << sha.shapes.at(shape_id).params[0],                   //normalized
          sha.shapes.at(shape_id).params[1],
          sha.shapes.at(shape_id).params[2];

      sha.shapes.at(shape_id).centroid.x = (float)feedback->pose.position.x ;
      sha.shapes.at(shape_id).centroid.y = (float)feedback->pose.position.y ;
      sha.shapes.at(shape_id).centroid.z = (float)feedback->pose.position.z ;

      newCentroid << sha.shapes.at(shape_id).centroid.x ,
          sha.shapes.at(shape_id).centroid.y ,
          sha.shapes.at(shape_id).centroid.z ;


      transSecondStep.block(0,0,3,3) << rotationMat ;
      transSecondStep.col(3).head(3) << newCentroid(0) , newCentroid(1), newCentroid(2) ;
      transSecondStep.row(3) << 0,0,0,1 ;

      Eigen::Affine3f affineSecondStep(transSecondStep) ;

      std::cout << "transfrom : " << "\n"    << affineSecondStep.matrix() << "\n" ;

      Eigen::Affine3f affineFinal(affineSecondStep*affineInit) ;
      Eigen::Matrix4f matFinal = (transSecondStep*transInitInv) ;

      vecNew    = (matFinal.block(0,0,3,3))* vec;
      //      newCentroid  = transFinal *OldCentroid ;


      sha.shapes.at(shape_id).centroid.x = newCentroid(0) ;
      sha.shapes.at(shape_id).centroid.y = newCentroid(1) ;
      sha.shapes.at(shape_id).centroid.z = newCentroid(2) ;


      sha.shapes.at(shape_id).params[0] = vecNew(0) ;
      sha.shapes.at(shape_id).params[1] = vecNew(1) ;
      sha.shapes.at(shape_id).params[2] = vecNew(2) ;


      std::cout << "transfromFinal : " << "\n"    << affineFinal.matrix() << "\n" ;

      pcl::PointCloud<pcl::PointXYZ> pc;
      pcl::PointXYZ pt;
      sensor_msgs::PointCloud2 pc2;

      for(unsigned int j=0; j<p.contours.size(); j++)
      {
        for(unsigned int k=0; k<p.contours[j].size(); k++)
        {
          p.contours[j][k] = affineFinal * p.contours[j][k];
          pt.x = p.contours[j][k][0] ;
          pt.y = p.contours[j][k][1] ;
          pt.z = p.contours[j][k][2] ;
          pc.push_back(pt) ;
        }
      }

      pcl::toROSMsg (pc, pc2);
      sha.shapes.at(shape_id).points.clear() ;
      sha.shapes.at(shape_id).points.push_back (pc2);

      // uncomment when using test_shape_array

      //      for(unsigned int i=0;i<sha.shapes.size();i++){
      //        map_msg.header = sha.shapes.at(i).header ;
      //        map_msg.shapes.push_back(sha.shapes.at(i)) ;
      //      }
      //      shape_pub_.publish(map_msg);

      // end uncomment

      req.InMap.shapes.push_back(sha.shapes.at(shape_id));
    }

  }
}

void ShapeMarker::deleteMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {

  stringstream ss;
  ss << shape_.id ;// ctr_ ;
  marker_.name = ss.str() ;
  std::cout << "Marker" << marker_.name << " deleted..."<< std::endl ;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  menu_handler_.apply (*im_server_, marker_.name);
  im_server_->applyChanges ();
  interactedShapes.push_back(shape_.id);
  //  interacted_shapes.push_back(check_state) ;
}


void ShapeMarker::displayContour(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  stringstream aa;
  stringstream ss;
  int ctr = 0;

  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration();
  marker.header = shape_.header ;
  marker.header.frame_id = "/map";
  marker.ns = "contours" ;

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 1;

  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1.0;

  cob_3d_mapping::Polygon p;
  cob_3d_mapping::fromROSMsg (shape_, p);

  interactive_markers::MenuHandler::CheckState check_state;

  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    ROS_INFO ("Displaying Contour...") ;
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);

    visualization_msgs::InteractiveMarker imarker;
    for(unsigned int i=0; i<p.contours.size(); i++)
    {
      marker.id = ctr ;
      std::cout << "marker id : " << marker.id  << std::endl;
      ctr ++ ;
      for(unsigned int j=0; j<p.contours[i].size(); j++)
      {
        marker.points.resize(p.contours[i].size()+1);

        marker.points[j].x = p.contours[i][j](0);
        marker.points[j].y = p.contours[i][j](1);
        marker.points[j].z = p.contours[i][j](2);
      }
      marker.points[p.contours[i].size()].x = p.contours[i][0](0);
      marker.points[p.contours[i].size()].y = p.contours[i][0](1);
      marker.points[p.contours[i].size()].z = p.contours[i][0](2);
    }

    // Interactive Marker for contours

    visualization_msgs::InteractiveMarkerControl im_ctrl_ ;
    im_ctrl_.always_visible = true ;    // if it is not set in to true the box is only visible when the interact button is pressed
    im_ctrl_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    ss << "contour_" << shape_.id;
    imarker.name = ss.str() ;

    imarker.header  = shape_.header ;
    im_ctrl_.markers.push_back(marker);
    imarker.controls.push_back(im_ctrl_);
    im_server_->insert (imarker);
    menu_handler_.apply (*im_server_, imarker.name);
    interactedShapes.push_back(shape_.id) ;
    interactedShapes.push_back(check_state) ;
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    ROS_INFO ("Deleting the Contour...");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    ss.clear() ;
    ss.str("");
    ss << "contour_" << shape_.id;
    im_server_->erase(ss.str());

    // if anything is unchecked, there is no need to reset it again!
    interactedShapes.pop_back() ;
    interactedShapes.pop_back() ;
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();
}

void ShapeMarker::getPosition (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  int flag = 0 ;
  //  ROS_INFO("In GetPosition function....");
  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);

  shape_.header = marker_.header ;
  shape_.header.frame_id = "/map" ;

  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    flag = 1;
    moveMarker(flag);
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    interactedShapes.push_back(shape_.id) ;
    interactedShapes.push_back(check_state) ;

  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    flag = 0;
    moveMarker(flag);
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);

    // if anything is unchecked, there is no need to reset it again!
    interactedShapes.pop_back() ;
    interactedShapes.pop_back() ;

  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges();
}


void ShapeMarker::moveMarker(int flag)
{

  visualization_msgs::InteractiveMarkerControl im_ctrl;
  stringstream ss;
  ss.str("");
  ss.clear();
  //  std::cout << "ss: \t" << ss.str() << "\n" ;

//  ss << "arrows_" << shape_.id ;
//  marker_.name = ss.str() ;
  marker_.header  = shape_.header ;
  marker_.header.frame_id = "/map" ;


  if (flag == 1) {
    ROS_INFO("Adding the arrows... ");
    //  Im_ctrl.always_visible = true;
    im_ctrl.name = "arrow_markers" ;

    im_ctrl.orientation.w = 1;
    im_ctrl.orientation.x = 1;
    im_ctrl.orientation.y = 0;
    im_ctrl.orientation.z = 0;
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker_.controls.push_back (im_ctrl);
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker_.controls.push_back (im_ctrl);

    im_ctrl.orientation.w = 1;
    im_ctrl.orientation.x = 0;
    im_ctrl.orientation.y = 1;
    im_ctrl.orientation.z = 0;
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker_.controls.push_back (im_ctrl);
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker_.controls.push_back (im_ctrl);

    im_ctrl.orientation.w = 1;
    im_ctrl.orientation.x = 0;
    im_ctrl.orientation.y = 0;
    im_ctrl.orientation.z = 1;
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker_.controls.push_back (im_ctrl);
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker_.controls.push_back (im_ctrl);

    // Save the ids of shapes that have been moved
    movedShapesIndexes.push_back(shape_.id) ;

    std::cout << "Name of the Marker arrow : \t" << marker_.name << "\n" ;

    im_server_->insert (marker_);
    im_server_->applyChanges() ;

  }

  else if (flag == 0)
  {
    ROS_INFO ("Deleting the Arrows ...") ;

    // Disabling MOVE_AXIS
    marker_.controls.pop_back() ;
    marker_.controls.pop_back() ;
    marker_.controls.pop_back() ;

    // Disabling ROTATE_AXIS
    marker_.controls.pop_back() ;
    marker_.controls.pop_back() ;
    marker_.controls.pop_back() ;


    im_server_->insert (marker_);
    im_server_->applyChanges() ;
  }

}

void ShapeVisualization::applyModifications(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{

  std ::cout << "size of request: " << req.InMap.shapes.size() << "\n" ;

  if (ros::service::call("geometry_map/modify_map",req,res))
  {
    std::cout << "calling ModifyMap service..." << "\n" ;
  }

  while (!req.InMap.shapes.empty()){
    req.InMap.shapes.pop_back() ;
  }
  std ::cout << "size of request: " << req.InMap.shapes.size() << "\n" ;

  im_server_->applyChanges() ;
}




void ShapeMarker::resetMarker(bool reset_marker,visualization_msgs::InteractiveMarker& imarker) {


  std::cout << "shape_ Name :" << shape_.id << "\n" ;

  stringstream aa;
  stringstream ss;

  interactive_markers::MenuHandler::CheckState Check_state;

  for (unsigned int j=1; j< imarker.menu_entries.size()-1;j++)
  {
    menu_handler_.setCheckState (imarker.menu_entries.at(j).id, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler_.apply(*im_server_,imarker.name) ;
    im_server_->applyChanges() ;

  }



  for (unsigned int i=0; i< interactedShapes.size();i+=2)
  {
    ss << "normal_" << interactedShapes.at(i);
    im_server_->erase(ss.str());
    ss.str("");
    ss.clear();
    //
    ss << "centroid_" <<  interactedShapes.at(i);
    im_server_->erase(ss.str());
    ss.str("");
    ss.clear();
    //
    ss << "contour_" << interactedShapes.at(i);
    im_server_->erase(ss.str());
    ss.str("");
    ss.clear();
    //
//    ss << "arrows_" << interactedShapes.at(i);
//    im_server_->erase(ss.str());
    ss.str("");
    ss.clear();
    im_server_->applyChanges ();
  }
}

void ShapeVisualization::resetAll(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  stringstream aa;


  for (unsigned int i=0; i< interactedShapes.size();i+=2){
    visualization_msgs::InteractiveMarker Imarker ;
    aa.str("");
    aa.clear();
    aa << interactedShapes.at(i) ;
    int j = interactedShapes.at(i) ;
    int ctr;

    std::cout << Imarker.name << "\n" ;
    im_server_->get(aa.str() , Imarker) ;
    boost::shared_ptr<ShapeMarker> shapeMarker(new ShapeMarker(im_server_, sha.shapes[i],ctr));
    //      ShapeMarker sm(im_server_,sha.shapes.at(j),ctr);
    shapeMarker->getShape(sha.shapes.at(j));
    shapeMarker->resetMarker(true,Imarker);

  }
  std::cout << "interacted_shapes size before unticking: "<< interactedShapes.size() << "\n";

  // Clearing the interacted_shapes vector for the next step
  while (!interactedShapes.empty()){
    interactedShapes.pop_back() ;
  }
  std::cout << "interacted_shapes size after unticking: "<< interactedShapes.size() << "\n";
  im_server_->applyChanges ();

}


void ShapeVisualization::moreOptions()
{
  optionMenu();
  visualization_msgs::Marker Text;

  Text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  Text.action = visualization_msgs::Marker::ADD;
  Text.lifetime = ros::Duration ();
  Text.header.frame_id = "/map" ;

  Text.id = 0;
  Text.ns = "text";

  Text.text = "Click here for more options" ;
  // Scale
  Text.scale.x = 0.2;
  Text.scale.y = 0.2;
  Text.scale.z = 0.2;

  // Pose
  Text.pose.position.x = 0;
  Text.pose.position.y = 0;
  Text.pose.position.z = 0.5;

  Text.pose.orientation.x = 0;
  Text.pose.orientation.y = 0;
  Text.pose.orientation.z = 0;
  Text.pose.orientation.w = 1;



  Text.color.r = 1;
  Text.color.g = 1;
  Text.color.b = 1;
  Text.color.a = 1;

  /*Interactive Marker for the Text*/

  visualization_msgs::InteractiveMarker imarkerText;
  visualization_msgs::InteractiveMarkerControl im_ctrl_text_ ;
  im_ctrl_text_.always_visible = true ;
  im_ctrl_text_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  imarkerText.name = "Text" ;
  imarkerText.header  = Text.header ;
  im_ctrl_text_.markers.push_back(Text);
  imarkerText.controls.push_back(im_ctrl_text_);

  im_server_->insert (imarkerText);
  menu_handler_for_text_.apply (*im_server_,imarkerText.name);

}

void ShapeVisualization::displayAllNormals(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  stringstream ss;
  interactive_markers::MenuHandler::CheckState check_state;
  std::cout << "SIZE OF THE SHAPE ARRAY: " << sha.shapes.size() ;

  menu_handler_for_text_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    ROS_INFO ("Displaying all Normals...");
    menu_handler_for_text_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);

    visualization_msgs::InteractiveMarker imarker;
    for (unsigned int i=0; i< sha.shapes.size();i++)
    {

      ss.str("");
      ss.clear() ;
      //      ROS_INFO("displaying normal for shape %d",sa_.shapes[i].id);
      ss << "normal_" << sha.shapes[i].id;

      imarker.name = ss.str();
      imarker.header = sha.shapes[i].header;
      ss.str("");
      ss.clear();
      //marker_id_ = 0;
      visualization_msgs::Marker marker;
      marker.header =sha.shapes[i].header;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration ();

      //set color
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1;

      //set scale
      marker.scale.x = 0.05;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      //set pose
      marker.points.resize (2);
      marker.points[0].x = sha.shapes[i].centroid.x;
      marker.points[0].y = sha.shapes[i].centroid.y;
      marker.points[0].z = sha.shapes[i].centroid.z;

      marker.points[1].x = sha.shapes[i].centroid.x + sha.shapes[i].params[0];
      marker.points[1].y = sha.shapes[i].centroid.y + sha.shapes[i].params[1];
      marker.points[1].z = sha.shapes[i].centroid.z + sha.shapes[i].params[2];


      //marker.id = shape_.id;

      visualization_msgs::InteractiveMarkerControl im_ctrl_n;

      im_ctrl_n.always_visible = true;
      ss << "normal_ctrl_" << sha.shapes[i].id;
      im_ctrl_n.name = ss.str ();
      im_ctrl_n.description = "display_normal";

      //im_ctrl_n.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      im_ctrl_n.markers.push_back (marker);
      imarker.controls.push_back (im_ctrl_n);
      im_server_->insert (imarker);
    }
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_for_text_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    ROS_INFO ("Deleting all Normals...");
    for (unsigned int i=0; i< sha.shapes.size();i++){
      ss << "normal_" << sha.shapes[i].id ;
      im_server_->erase(ss.str());
      ss.str("");
      ss.clear();
    }
  }
  menu_handler_for_text_.reApply (*im_server_);
  im_server_->applyChanges ();
}

void
ShapeVisualization::displayAllCentroids (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  //  ROS_INFO(" displayCentroidCB  of shape[ %d ]...", shape_.id);

  stringstream ss;
  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_for_text_.getCheckState (feedback->menu_entry_id, check_state);
  std::cout << feedback->menu_entry_id << "\n" ;

  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    menu_handler_for_text_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    visualization_msgs::InteractiveMarker imarker;
    for (unsigned int i=0; i< sha.shapes.size();i++)
    {
      ss.str("");
      ss.clear() ;
      ss << "centroid_" << sha.shapes[i].id;

      imarker.name = ss.str();
      imarker.header = sha.shapes[i].header;
      ss.str("");
      ss.clear();

      visualization_msgs::Marker marker;
      //marker.id = shape_.id;
      marker.header = sha.shapes[i].header;

      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration ();

      //set color
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 1;

      //set scale
      marker.scale.x = 0.04;
      marker.scale.y = 0.04;
      marker.scale.z = 0.04;
      //std::cout << " shape index : " << shape_idx << std::endl;

      //set pose
      marker.pose.position.x = sha.shapes[i].centroid.x;
      marker.pose.position.y = sha.shapes[i].centroid.y;
      marker.pose.position.z = sha.shapes[i].centroid.z;


      visualization_msgs::InteractiveMarkerControl im_ctrl;
      im_ctrl.always_visible = true;
      ss << "centroid_ctrl_" << sha.shapes[i].id;
      im_ctrl.name = ss.str ();
      im_ctrl.markers.push_back (marker);
      imarker.controls.push_back (im_ctrl);
      im_server_->insert (imarker);
    }
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    menu_handler_for_text_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    for (unsigned int i=0; i< sha.shapes.size();i++)
    {

      //ROS_INFO(" entry state changed ");
      ss << "centroid_" << sha.shapes[i].id;
      im_server_->erase(ss.str());
      ss.str("");
      ss.clear();
    }
  }
  menu_handler_for_text_.reApply (*im_server_);
  im_server_->applyChanges ();
}


void
ShapeMarker::createShapeMenu ()
{
  //ROS_INFO(" creating menu .....");

  interactive_markers::MenuHandler::EntryHandle eh_1, eh_2, eh_3, eh_4, eh_5, eh_7;

  eh_1 = menu_handler_.insert ("Menu");
  eh_2 = menu_handler_.insert (eh_1, "Display Normal",boost::bind (&ShapeMarker::displayNormalCB, this, _1));
  eh_3 = menu_handler_.insert (eh_1, "Display Centroid",boost::bind (&ShapeMarker::displayCentroidCB, this, _1));
  eh_4 = menu_handler_.insert (eh_1, "Display Contour",boost::bind (&ShapeMarker::displayContour, this, _1));
  eh_5 = menu_handler_.insert (eh_1, "Enable Movement",boost::bind (&ShapeMarker::getPosition, this, _1));
  eh_7 = menu_handler_.insert (eh_1, "Delete Marker",boost::bind (&ShapeMarker::deleteMarker, this, _1));
  //    eh_6 = menu_handler_.insert (eh_1, "Fix to this Position",boost::bind (&ShapeMarker::setShapePosition, this, _1));

  menu_handler_.setVisible (eh_1, true);
  menu_handler_.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);

  menu_handler_.setVisible (eh_2, true);
  menu_handler_.setCheckState (eh_2, interactive_markers::MenuHandler::UNCHECKED);

  menu_handler_.setVisible (eh_3, true);
  menu_handler_.setCheckState (eh_3, interactive_markers::MenuHandler::UNCHECKED);

  menu_handler_.setVisible (eh_4, true);
  menu_handler_.setCheckState (eh_4, interactive_markers::MenuHandler::UNCHECKED);

  menu_handler_.setVisible (eh_5, true);
  menu_handler_.setCheckState (eh_5, interactive_markers::MenuHandler::UNCHECKED);

  menu_handler_.setVisible (eh_7, true);
  menu_handler_.setCheckState (eh_7, interactive_markers::MenuHandler::NO_CHECKBOX);

}

void ShapeVisualization::optionMenu() {

  //  ROS_INFO("Creating menu for the box...") ;

  interactive_markers::MenuHandler::EntryHandle eh_1, eh_2 , eh_3 ,eh_4 , eh_5;

  eh_1 = menu_handler_for_text_.insert ("Menu");
  eh_2 = menu_handler_for_text_.insert (eh_1, "All Normals",boost::bind (&ShapeVisualization::displayAllNormals, this, _1));
  eh_3 = menu_handler_for_text_.insert (eh_1, "All Centroids",boost::bind (&ShapeVisualization::displayAllCentroids, this, _1));
  eh_4 = menu_handler_for_text_.insert (eh_1, "Apply map modifications",boost::bind (&ShapeVisualization::applyModifications, this, _1));
  eh_5 = menu_handler_for_text_.insert (eh_1, "Reset all Controls",boost::bind (&ShapeVisualization::resetAll, this, _1));

  menu_handler_for_text_.setVisible (eh_1, true);
  menu_handler_for_text_.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);
  menu_handler_for_text_.setVisible (eh_2, true);
  menu_handler_for_text_.setCheckState (eh_2, interactive_markers::MenuHandler::UNCHECKED);
  menu_handler_for_text_.setVisible (eh_3, true);
  menu_handler_for_text_.setCheckState (eh_3, interactive_markers::MenuHandler::UNCHECKED);
  menu_handler_for_text_.setVisible (eh_4, true);
  menu_handler_for_text_.setCheckState (eh_4, interactive_markers::MenuHandler::NO_CHECKBOX);
  menu_handler_for_text_.setVisible (eh_5, true);
  menu_handler_for_text_.setCheckState (eh_5, interactive_markers::MenuHandler::NO_CHECKBOX);


}



/**
 * @brief Create marker for the shape and add it to the interactive marker control
 *
 * @param triangle_list triangulated list of poly points
 * @param im_ctrl interactive marker control
 * @param transformation transformation matrix for this shape
 *
 */
void
ShapeMarker::createMarker (list<TPPLPoly>& triangle_list, visualization_msgs::InteractiveMarkerControl& im_ctrl)
{
  //ROS_INFO(" creating markers for this shape.....");
  TPPLPoint pt;
  for (std::list<TPPLPoly>::iterator it = triangle_list.begin (); it != triangle_list.end (); it++)
  {
    marker.id = shape_.id;

    marker.header = shape_.header;

    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.ns = "shape visualization";
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration ();

    //set color
    marker.color.r = shape_.color.r;
    marker.color.g = shape_.color.g;
    marker.color.b = shape_.color.b;
    marker.color.a = shape_.color.a;

    //set scale
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    //set pose
    Eigen::Quaternionf quat (transformation_inv_.rotation ());
    Eigen::Vector3f trans (transformation_inv_.translation ());

    marker.pose.position.x = trans (0);
    marker.pose.position.y = trans (1);
    marker.pose.position.z = trans (2);

    marker.pose.orientation.x = quat.x ();
    marker.pose.orientation.y = quat.y ();
    marker.pose.orientation.z = quat.z ();
    marker.pose.orientation.w = quat.w ();

    //draw each triangle
    marker.points.resize (it->GetNumPoints ());
    for (long i = 0; i < it->GetNumPoints (); i++)
    {
      pt = it->GetPoint (i);
      marker.points[i].x = pt.x;
      marker.points[i].y = pt.y;
      marker.points[i].z = 0;
    }
    im_ctrl.markers.push_back (marker);
  }


  // Added For displaying the arrows on Marker Position
  marker_.pose.position.x = marker.pose.position.x ;
  marker_.pose.position.y = marker.pose.position.y ;
  marker_.pose.position.z = marker.pose.position.z ;

  marker_.pose.orientation.x = marker.pose.orientation.x ;
  marker_.pose.orientation.y = marker.pose.orientation.y ;
  marker_.pose.orientation.z = marker.pose.orientation.z ;
  // end

}

/**
 * @brief Create menu entries for each shape
 *
 * @param point 3D point to be transformed
 * @param transformation transformation matrix for this shape
 *
 * @return return transformed 2D TPPLPoint
 */
TPPLPoint
ShapeMarker::msgToPoint2D (const pcl::PointXYZ &point)
{
  //ROS_INFO(" transform 3D point to 2D ");
  TPPLPoint pt;
  Eigen::Vector3f p = transformation_ * point.getVector3fMap ();
  pt.x = p (0);
  pt.y = p (1);
  //std::cout << "\n transformed point : \n" << p << std::endl;
  return pt;
}

/**
 * @brief Publish interactive markers for a shape message using interactive marker server
 *
 * @param shape_msg Shape message for which the interactive marker is to be created
 *
 */
void
ShapeMarker::createInteractiveMarker ()
{
  //    ROS_INFO("\tcreating interactive marker for shape < %d >", shape_.id);

  /* get normal and centroid */
  cob_3d_mapping::Polygon p;
  cob_3d_mapping::fromROSMsg (shape_, p);

  /* transform shape points to 2d and store 2d point in triangle list */
  TPPLPartition pp;
  list<TPPLPoly> polys, tri_list;

  Eigen::Vector3f v, normal, origin;
  if (shape_.params.size () == 4)
  {
    normal (0) = shape_.params[0];
    normal (1) = shape_.params[1];
    normal (2) = shape_.params[2];
    origin (0) = shape_.centroid.x;
    origin (1) = shape_.centroid.y;
    origin (2) = shape_.centroid.z;
    v = normal.unitOrthogonal ();

    pcl::getTransformationFromTwoUnitVectorsAndOrigin (v, normal, origin, transformation_);
    transformation_inv_ = transformation_.inverse ();
  }

  for (size_t i = 0; i < shape_.points.size (); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> pc;
    TPPLPoly poly;
    pcl::fromROSMsg (shape_.points[i], pc);
    poly.Init (pc.points.size ());
    poly.SetHole (shape_.holes[i]);

    for (size_t j = 0; j < pc.points.size (); j++)
    {
      poly[j] = msgToPoint2D (pc[j]);
    }
    if (shape_.holes[i])
      poly.SetOrientation (TPPL_CW);
    else
      poly.SetOrientation (TPPL_CCW);

    polys.push_back (poly);
  }
  pp.Triangulate_EC (&polys, &tri_list);

  /* create interactive marker for *this shape */
  stringstream ss;
  ss << shape_.id ;
  marker_.name = ss.str ();
  marker_.header = shape_.header;

  ss.str ("");
  im_ctrl.always_visible = true;
  ss << "shape_" << shape_.id << "_control";
  im_ctrl.name = ss.str ();
  im_ctrl.description = "shape_markers";
  im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;


  /* create marker */
  createMarker (tri_list, im_ctrl);


  marker_.controls.push_back (im_ctrl);
  im_server_->insert (marker_ );
  /* create menu for *this shape */
  im_server_ ->applyChanges() ;
  menu_handler_.apply (*im_server_, marker_.name);

}


/**
 * @brief Feedback callback for normal menu entry
 *
 * @param feedback feedback from rviz when the normal menu entry of a shape is changed
 * @param shape_idx index of shape from which the feedback is received
 * @param menu_h_ptr pointer to menu entries of this shape
 * @param transformation matrix for this shape
 */
void
ShapeMarker::displayNormalCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO(" displayNormalCB from shape[ %d ]...", shape_.id);

  stringstream ss;
  interactive_markers::MenuHandler::CheckState check_state;

  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    std::cout << "feedback->menu_entry_id : "<< feedback->menu_entry_id << "\n" ;
    visualization_msgs::InteractiveMarker imarker;
    ss << "normal_" << shape_.id;
    imarker.name = ss.str();
    imarker.header = shape_.header;
    ss.str("");
    ss.clear();

    visualization_msgs::Marker marker;
    marker.header = shape_.header;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration ();

    //set color
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    //set scale
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    //set pose
    marker.points.resize (2);
    marker.points[0].x = shape_.centroid.x;
    marker.points[0].y = shape_.centroid.y;
    marker.points[0].z = shape_.centroid.z;

    marker.points[1].x = shape_.centroid.x + shape_.params[0];
    marker.points[1].y = shape_.centroid.y + shape_.params[1];
    marker.points[1].z = shape_.centroid.z + shape_.params[2];

    visualization_msgs::InteractiveMarkerControl im_ctrl_n;

    ss << "normal_ctrl_" << shape_.id;
    im_ctrl_n.name = ss.str ();
    im_ctrl_n.description = "display_normal";

    //im_ctrl_n.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    im_ctrl_n.markers.push_back (marker);
    imarker.controls.push_back (im_ctrl_n);
    im_server_->insert (imarker);
    interactedShapes.push_back(shape_.id) ;
    interactedShapes.push_back(feedback->menu_entry_id) ;
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    ss << "normal_" << shape_.id;
    im_server_->erase(ss.str());

    // if anything is unchecked, there is no need to reset it again!
    interactedShapes.pop_back() ;
    interactedShapes.pop_back() ;
    std::cout << "interacted_shapes size : "<< interactedShapes.size() << "\n";

  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();

}
/**
 * @brief Feedback callback for centroid menu entry
 *
 * @param feedback feedback from rviz when the centroid menu entry of a shape is changed
 * @param shape_idx index of shape from which the feedback is received
 * @param menu_h_ptr pointer to menu entries of this shape
 */
void
ShapeMarker::displayCentroidCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO(" displayCentroidCB from shape[ %d ]...", shape_.id);
  stringstream ss;
  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    visualization_msgs::InteractiveMarker imarker;
    ss << "centroid_" << shape_.id;
    imarker.name = ss.str();
    imarker.header = shape_.header;
    ss.str("");
    ss.clear();

    visualization_msgs::Marker marker;
    marker.header = shape_.header;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration ();

    //set color
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1;

    //set scale
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    //set pose
    marker.pose.position.x = shape_.centroid.x;
    marker.pose.position.y = shape_.centroid.y;
    marker.pose.position.z = shape_.centroid.z;


    visualization_msgs::InteractiveMarkerControl im_ctrl;
    im_ctrl.always_visible = true;
    ss << "centroid_ctrl_" << shape_.id;
    im_ctrl.name = ss.str ();
    im_ctrl.markers.push_back (marker);
    imarker.controls.push_back (im_ctrl);
    im_server_->insert (imarker);

    interactedShapes.push_back(shape_.id) ;
    interactedShapes.push_back(feedback->menu_entry_id) ;

  }
  if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    ss << "centroid_" << shape_.id;
    im_server_->erase(ss.str());

    // if anything is unchecked, there is no need to reset it again!
    interactedShapes.pop_back() ;
    interactedShapes.pop_back() ;

  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();


}

void
ShapeVisualization::shapeArrayCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& sa)
{
  ctr_for_shape_indexes = 0 ;
  //  v_sm_.clear();
  sha.shapes.clear() ;
  im_server_->applyChanges();
  ROS_INFO("shape array with %d shapes received", sa->shapes.size());

  for (unsigned int i = 0; i < sa->shapes.size (); i++)
  {
    sha.shapes.push_back(sa->shapes[i]);
    sha.shapes[i].id = i;
    boost::shared_ptr<ShapeMarker> sm(new ShapeMarker(im_server_, sa->shapes[i],ctr_for_shape_indexes));
    v_sm_.push_back(sm);
  }
  im_server_->applyChanges(); //update changes
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "shape_visualization");
  ROS_INFO("shape_visualization node started....");
  ShapeVisualization sv;
  ros::spin();
}
