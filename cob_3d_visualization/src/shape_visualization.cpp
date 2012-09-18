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



// Global variables needed for ModifyMap service
cob_3d_mapping_msgs::ModifyMap::Request req ;
cob_3d_mapping_msgs::ModifyMap::Response res;
Eigen::Quaternionf quatInit ;
Eigen::Vector3f oldCentroid ;
Eigen::Matrix4f transInit;
Eigen::Affine3f affineInit;
Eigen::Matrix4f transInitInv;


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

void ShapeVisualization::resetAll(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  stringstream aa;
  boost::shared_ptr<ShapeMarker> shapeMarker(new ShapeMarker(im_server_, sha.shapes[0]));

  std::vector<int> interactedShapes = shapeMarker->getInteractedShapesNumber();

  for (unsigned int i=0; i< interactedShapes.size();i+=2){
    visualization_msgs::InteractiveMarker Imarker ;
    aa.str("");
    aa.clear();
    aa << interactedShapes.at(i) ;
    int j = interactedShapes.at(i) ;
    //    int ctr;

    std::cout << Imarker.name << "\n" ;
    im_server_->get(aa.str() , Imarker) ;
    boost::shared_ptr<ShapeMarker> shapeMarker(new ShapeMarker(im_server_, sha.shapes[i]));
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


      visualization_msgs::InteractiveMarkerControl im_ctrl_n;

      im_ctrl_n.always_visible = true;
      ss << "normal_ctrl_" << sha.shapes[i].id;
      im_ctrl_n.name = ss.str ();
      im_ctrl_n.description = "display_normal";

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


void ShapeVisualization::optionMenu() {

  //  ROS_INFO("Creating menu for the text...") ;

  interactive_markers::MenuHandler::EntryHandle eh_1, eh_2 , eh_3 ,eh_4 , eh_5, eh_6;

  eh_1 = menu_handler_for_text_.insert ("Menu");
  eh_2 = menu_handler_for_text_.insert (eh_1, "All Normals",boost::bind (&ShapeVisualization::displayAllNormals, this, _1));
  eh_3 = menu_handler_for_text_.insert (eh_1, "All Centroids",boost::bind (&ShapeVisualization::displayAllCentroids, this, _1));
  eh_4 = menu_handler_for_text_.insert (eh_1, "Find tables",boost::bind (&ShapeVisualization::findTables, this, _1));
  eh_5 = menu_handler_for_text_.insert (eh_1, "Apply map modifications",boost::bind (&ShapeVisualization::applyModifications, this, _1));
  eh_6 = menu_handler_for_text_.insert (eh_1, "Reset all Controls",boost::bind (&ShapeVisualization::resetAll, this, _1));

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
  menu_handler_for_text_.setVisible (eh_6, true);
  menu_handler_for_text_.setCheckState (eh_6, interactive_markers::MenuHandler::NO_CHECKBOX);


}

void ShapeVisualization::findTables(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback){
  //  cob_3d_mapping_msgs::GetTables::Request req ;
  //  cob_3d_mapping_msgs::GetTables::Response res;

  cob_3d_mapping_msgs::GetObjectsOfClass::Request req;
  cob_3d_mapping_msgs::GetObjectsOfClass::Response res;
  std::vector<geometry_msgs::Pose> tablePose ;
  stringstream aa;


  //  interactive_markers::MenuHandler::CheckState check_state;
  //  menu_handler_for_text_.getCheckState (feedback->menu_entry_id, check_state);

  //  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  //  {
  //    menu_handler_for_text_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
  if (ros::service::call("/table_extraction/get_objects_of_class",req,res))        ///table_extraction/get_tables
  {
    std::cout << "calling GetTables service..." << "\n" ;
    //      shape_pub_.publish(sha);

  }


  tablePose.resize(res.objects.shapes.size()) ;
  visualization_msgs::InteractiveMarker interactiveMarker;

  for (unsigned int i=0;i<res.objects.shapes.size();i++){

    //    shapeMarker->getShape(res.objects.shapes[i]);
    aa.str("");
    aa.clear();
    aa << res.objects.shapes[i].id ;
    interactiveMarker.name = aa.str() ;
//    im_server_->erase(aa.str()) ;
//    im_server_->applyChanges();
//    ros::Duration(2).sleep() ;

//    boost::shared_ptr<ShapeMarker> shapeMarker(new ShapeMarker(im_server_, res.objects.shapes[i]));

    tablePose[i].position.x = res.objects.shapes[i].centroid.x;
    tablePose[i].position.y = res.objects.shapes[i].centroid.y;
    tablePose[i].position.z = res.objects.shapes[i].centroid.z;
    ROS_INFO("Position of the Table[%d]: x:%f , y:%f , z:%f", i, tablePose[i].position.x, tablePose[i].position.y, tablePose[i].position.z);
  }

  //  boost::shared_ptr<ShapeMarker> shapeMarker(new ShapeMarker(im_server_, res.objects.shapes[i],i));
  //      //      ShapeMarker sm(im_server_,sha.shapes.at(j),ctr);
  //      shapeMarker->getShape(sha.shapes.at(j));
  //      shapeMarker->resetMarker(true,Imarker);


  //  }
  //  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  //  {
  //    menu_handler_for_text_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
  //    res.tables.clear();
  //  }
  //  menu_handler_for_text_.reApply (*im_server_);
  //  im_server_->applyChanges ();
}

void
ShapeVisualization::shapeArrayCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& sa)
{
  //  ctr_for_shape_indexes = 0 ;
  v_sm_.clear();
  sha.shapes.clear() ;
  im_server_->applyChanges();
  ROS_INFO("shape array with %d shapes received", sa->shapes.size());

  for (unsigned int i = 0; i < sa->shapes.size (); i++)
  {
    sha.shapes.push_back(sa->shapes[i]);
    sha.shapes[i].id = i;
    ROS_INFO("id of Shape[%d] is: %d", i,sha.shapes[i].id);
    boost::shared_ptr<ShapeMarker> sm(new ShapeMarker(im_server_, sa->shapes[i]));
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
