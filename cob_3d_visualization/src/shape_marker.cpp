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
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_common
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 09/2012
 *
 * \brief
 * Description:
 *
 * ToDo:
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

//##################
//#### includes ####

#include <cob_3d_visualization/shape_marker.h>
#include <pcl/common/transforms.h>

ShapeMarker::ShapeMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server,
    cob_3d_mapping_msgs::Shape& shape,std::vector<unsigned int>& moved_shapes_indices,std::vector<unsigned int>& interacted_shapes,
    std::vector<unsigned int>& deleted_markers_indices, bool arrows,bool deleted) ://, unsigned int& deleted) :
    interacted_shapes_(interacted_shapes) , moved_shapes_indices_(moved_shapes_indices) , deleted_markers_indices_(deleted_markers_indices)
{
  arrows_ = arrows ;
  deleted_ = deleted ;
  im_server_ = im_server;
  shape_ = shape;
  id_ = shape.id;
  createShapeMenu ();
  createInteractiveMarker();
}

void
ShapeMarker::triangle_refinement(std::list<TPPLPoly>& i_list, std::list<TPPLPoly>& o_list){
  int n_circle = 20;

  TPPLPoly tri_new,tri_temp;
  TPPLPoint ptM,ptM01,ptM12,ptM20;
  for (std::list<TPPLPoly>::iterator it = i_list.begin (); it != i_list.end (); it++){
    int n[4]={0,1,2,0};

    ptM.x =(it->GetPoint(n[0]).x+it->GetPoint(n[1]).x+it->GetPoint(n[2]).x)/3;
    ptM.y =(it->GetPoint(n[0]).y+it->GetPoint(n[1]).y+it->GetPoint(n[2]).y)/3;

    ptM01.x=(it->GetPoint(n[0]).x+it->GetPoint(n[1]).x)/2;
    ptM01.y=(it->GetPoint(n[0]).y+it->GetPoint(n[1]).y)/2;

    ptM12.x=(it->GetPoint(n[1]).x+it->GetPoint(n[2]).x)/2;
    ptM12.y=(it->GetPoint(n[1]).y+it->GetPoint(n[2]).y)/2;

    ptM20.x=(it->GetPoint(n[2]).x+it->GetPoint(n[3]).x)/2;
    ptM20.y=(it->GetPoint(n[2]).y+it->GetPoint(n[3]).y)/2;

    tri_temp.Triangle(ptM01,ptM12,ptM20);


    double thresh = shape_.params[9]/6;
    if(fabs(it->GetPoint(n[0]).x-ptM.x)>thresh || fabs(it->GetPoint(n[1]).x-ptM.x)>thresh || fabs(it->GetPoint(n[2]).x-ptM.x)>thresh){
      //for every old triangle 6! new triangles are created
      for (long i = 0; i < it->GetNumPoints (); i++){
        tri_new.Triangle(tri_temp.GetPoint(n[i]),ptM,it->GetPoint(n[i]));
        //push new triangle in trinagle list
        o_list.push_back(tri_new);
        tri_new.Triangle(tri_temp.GetPoint(n[i]),it->GetPoint(n[i+1]),ptM);
        //push new triangle in trinagle list
        o_list.push_back(tri_new);
      }
    }
    else{
      tri_new.Triangle(it->GetPoint(n[0]),it->GetPoint(n[1]),it->GetPoint(n[2]));
      o_list.push_back(tri_new);
    }
  }
}

void ShapeMarker::getShape (cob_3d_mapping_msgs::Shape& shape) {
  shape_ = shape ;
}

bool ShapeMarker::getArrows (){
  return arrows_ ;
}

bool ShapeMarker::setArrows (){
  arrows_ = false ;
  return arrows_ ;
}

bool ShapeMarker::getDeleted (){
  return deleted_ ;
}

bool ShapeMarker::setDeleted (){
  deleted_ = false ;
  return deleted_ ;
}

unsigned int ShapeMarker::getID(){
  return id_;
}

void ShapeMarker::deleteMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {

  deleted_ = true ;

  std::stringstream ss;
  ss << shape_.id ;// ctr_ ;
  deleted_markers_indices_.push_back(shape_.id) ;

  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.name = ss.str() ;

  std::cout << "Marker" << interactiveMarker.name << " deleted..."<< std::endl ;


  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  // create a transparent interactive marker
  createInteractiveMarker () ;
  im_server_->applyChanges ();

}


void ShapeMarker::enableMovement (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);

  shape_.header = marker_.header ;
  //shape_.header.frame_id = "/map" ;

  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //    im_server_->setCallback(marker_.name,boost::bind (&ShapeMarker::setShapePosition, this, _1)
    //                              ,visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    displayArrows();
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    hideArrows(1);
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges();
}


void ShapeMarker::displayArrows()
{
  arrows_ = true ;
  moved_shapes_indices_.push_back(shape_.id) ;

  /*Creating a transparent marker at the original position*/
  createInteractiveMarker () ;
  /**/

  visualization_msgs::InteractiveMarkerControl im_ctrl;

  //    stringstream ss;
  //    ss.str("");
  //    ss.clear();

  //  ss << "arrows_" << shape_.id;
  //  marker_.name = ss.str() ;

  ROS_INFO("Adding the arrows... ");
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


  im_server_->insert (marker_);
  im_server_->applyChanges() ;


}


void ShapeMarker::hideArrows(int untick)

{
  arrows_ = false ;
  std::stringstream ss;
  std::stringstream aa;
  std::vector<unsigned int>::iterator iter;

  /**delete the arrows**/
  if(marker_.controls.size()>1){
    ROS_INFO ("Deleting the Arrows ...") ;
    marker_.controls.erase(marker_.controls.end()-6,marker_.controls.end()) ;
    im_server_->insert(marker_) ;
    //    im_server_->applyChanges() ;

    if (!untick){ // when the ResetAll option is used
      menu_handler_.setCheckState (5, interactive_markers::MenuHandler::UNCHECKED);
      menu_handler_.reApply (*im_server_);
      //      im_server_->applyChanges() ;
    }

    /**deleting the Transparent Marker**/
    ss.str("");
    ss.clear();
    ss << "second_marker_"<< shape_.id;
    im_server_->erase(ss.str());

    im_server_->applyChanges() ;
    /**end**/
  }
  if (untick){
    iter = find (moved_shapes_indices_.begin(), moved_shapes_indices_.end(), shape_.id) ;
    if (iter!=moved_shapes_indices_.end()){
      moved_shapes_indices_.erase(moved_shapes_indices_.begin()+(iter-moved_shapes_indices_.begin())) ;
    }
  }
}

void ShapeMarker::resetMarker()
{

  std::stringstream aa;
  std::stringstream ss;

  hideNormal(0);
  hideCentroid(0);
  hideContour(0);
  if (shape_.type == cob_3d_mapping_msgs::Shape::CYLINDER){
    hideSymAxis(0);
    hideOrigin(0);
  }
}

  //  interacted_shapes_.pop_back() ;

void
ShapeMarker::createShapeMenu ()
{
  //  ROS_INFO(" creating menu .....");

  interactive_markers::MenuHandler::EntryHandle eh_1, eh_2, eh_3, eh_4, eh_5, eh_6, eh_7;

  eh_1 = menu_handler_.insert ("Menu");
  eh_2 = menu_handler_.insert (eh_1, "Display Normal",boost::bind (&ShapeMarker::displayNormalCB, this, _1));
  eh_3 = menu_handler_.insert (eh_1, "Display Centroid",boost::bind (&ShapeMarker::displayCentroidCB, this, _1));
  eh_4 = menu_handler_.insert (eh_1, "Display Contour",boost::bind (&ShapeMarker::displayContourCB, this, _1));
  eh_7 = menu_handler_.insert (eh_1, "Show ID",boost::bind (&ShapeMarker::displayIDCB, this, _1));
  eh_5 = menu_handler_.insert (eh_1, "Enable Movement",boost::bind (&ShapeMarker::enableMovement, this, _1));
  eh_6 = menu_handler_.insert (eh_1, "Delete Marker",boost::bind (&ShapeMarker::deleteMarker, this, _1));
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

  menu_handler_.setVisible (eh_6, true);
  menu_handler_.setCheckState (eh_6, interactive_markers::MenuHandler::NO_CHECKBOX);

  menu_handler_.setVisible (eh_7, true);
  menu_handler_.setCheckState (eh_7, interactive_markers::MenuHandler::UNCHECKED);


  if(shape_.type==cob_3d_mapping_msgs::Shape::CYLINDER){
    interactive_markers::MenuHandler::EntryHandle eh_7,eh_8;

    eh_7 = menu_handler_.insert (eh_1, "Show Symmetry Axis",boost::bind (&ShapeMarker::displaySymAxisCB, this, _1));
    menu_handler_.setVisible (eh_7, true);
    menu_handler_.setCheckState (eh_7, interactive_markers::MenuHandler::UNCHECKED);

    eh_8 = menu_handler_.insert (eh_1, "Show Cylinder Origin",boost::bind (&ShapeMarker::displayOriginCB, this, _1));
    menu_handler_.setVisible (eh_8,true);
    menu_handler_.setCheckState (eh_8, interactive_markers::MenuHandler::UNCHECKED);
  }
}

void
ShapeMarker::createMarker (visualization_msgs::InteractiveMarkerControl& im_ctrl)
{
  marker.id = shape_.id;

  marker.header = shape_.header;
  //std::cout << marker.header.frame_id << std::endl;
  //marker.header.stamp = ros::Time::now() ;

  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.ns = "shape visualization";
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration ();

  //set color
  marker.color.g = shape_.color.g;
  marker.color.b = shape_.color.b;
  marker.color.r = shape_.color.r;
  if (arrows_ || deleted_){
    marker.color.a = 0.5;
  }
  else
  {
    marker.color.a = shape_.color.a;
    //      marker.color.r = shape_.color.r;
  }

  //set scale
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  /* transform shape points to 2d and store 2d point in triangle list */
  TPPLPartition pp;
  std::list<TPPLPoly> polys, tri_list;

  Eigen::Vector3f v, normal, origin;

  if(shape_.type== cob_3d_mapping_msgs::Shape::CYLINDER)
  {
    cob_3d_mapping::Cylinder c;
    cob_3d_mapping::fromROSMsg (shape_, c);
    /*for(unsigned int i=0; i<c.contours_[0].size(); i++)
    {
      std::cout << c.contours_[0][i](0) << "," << c.contours_[0][i](1) << std::endl;
    }*/
    c.triangulate(tri_list);
    /*c.ParamsFromShapeMsg();
    // make trinagulated cylinder strip
    //transform cylinder in local coordinate system
    c.makeCyl2D();
    c.TransformContours(c.transform_from_world_to_plane);
    //c.transform2tf(c.transform_from_world_to_plane);
    //TODO: WATCH OUT NO HANDLING FOR MULTY CONTOUR CYLINDERS AND HOLES
    TPPLPoly poly;
    TPPLPoint pt;


    for(size_t j=0;j<c.contours.size();j++){

      poly.Init(c.contours[j].size());
      poly.SetHole (shape_.holes[j]);


      for(size_t i=0;i<c.contours[j].size();++i){

        pt.x=c.contours[j][i][0];
        pt.y=c.contours[j][i][1];

        poly[i]=pt;

      }
      if (shape_.holes[j])
        poly.SetOrientation (TPPL_CW);
      else
        poly.SetOrientation (TPPL_CCW);
      polys.push_back(poly);
    }
    // triangualtion itno monotone triangles
    pp.Triangulate_EC (&polys, &tri_list);

    transformation_inv_ = c.transform_from_world_to_plane.inverse();
    // optional refinement step
    list<TPPLPoly> refined_tri_list;
    triangle_refinement(tri_list,refined_tri_list);
    tri_list=refined_tri_list;*/

  }
  else if(shape_.type== cob_3d_mapping_msgs::Shape::POLYGON)
  {
    cob_3d_mapping::Polygon p;

    if (shape_.params.size () == 4)
    {
      cob_3d_mapping::fromROSMsg (shape_, p);
      /*normal (0) = shape_.params[0];
      normal (1) = shape_.params[1];
      normal (2) = shape_.params[2];
      origin (0) = shape_.centroid.x;
      origin (1) = shape_.centroid.y;
      origin (2) = shape_.centroid.z;
      v = normal.unitOrthogonal ();

      pcl::getTransformationFromTwoUnitVectorsAndOrigin (v, normal, origin, transformation_);
      transformation_inv_ = transformation_.inverse ();*/
    }
    else
    {
      ROS_WARN("Unsupported polygon type, aborting...");
      return;
    }
    p.triangulate(tri_list);
    /*for (size_t i = 0; i < shape_.points.size (); i++)
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
    pp.Triangulate_EC (&polys, &tri_list);*/

  }//Polygon
  else
	ROS_WARN("Shape type is not supported %d", shape_.type);

  if(tri_list.size() == 0)
  {
    ROS_WARN("Could not triangulate, will not display this shape! (ID: %d)", shape_.id);
    return;
  }
  //ROS_INFO(" creating markers for this shape.....");

  marker.points.resize (/*it->GetNumPoints ()*/tri_list.size()*3);
  TPPLPoint pt;
  int ctr=0;
  for (std::list<TPPLPoly>::iterator it = tri_list.begin (); it != tri_list.end (); it++)
  {

    //draw each triangle
    switch(shape_.type)
    {
      case(cob_3d_mapping_msgs::Shape::POLYGON):
      {
        for (long i = 0; i < it->GetNumPoints (); i++)
        {
          pt = it->GetPoint (i);
          marker.points[3*ctr+i].x = pt.x;
          marker.points[3*ctr+i].y = pt.y;
          marker.points[3*ctr+i].z = 0;
          //if(shape_.id == 39) std::cout << pt.x << "," << pt.y << std::endl;
        }
        //std::cout << marker.points.size() << std::endl;
        break;
      }
      case(cob_3d_mapping_msgs::Shape::CYLINDER):
      {
        for (long i = 0; i < it->GetNumPoints (); i++)
        {
          pt = it->GetPoint(i);

          //apply rerolling of cylinder analogous to cylinder class
          /*if(shape_.params.size()!=10){
            break;
          }*/
          double r = shape_.params[3];
          double alpha = pt.x / r;

          marker.points[3*ctr+i].x = sin(alpha) * r;
          marker.points[3*ctr+i].y = pt.y;
          marker.points[3*ctr+i].z = cos(alpha) * r;

          ////Keep Cylinder flat - Debuging
          //marker.points[i].x = pt.x;
          //marker.points[i].y = pt.y;
          //marker.points[i].z = 0;
        }
        break;
      }
    }
    ctr++;
  }
  //set pose
  marker.pose = shape_.pose;
  /*Eigen::Quaternionf quat (transformation_inv_.rotation ());
  Eigen::Vector3f trans (transformation_inv_.translation ());

  marker.pose.position.x = trans (0);
  marker.pose.position.y = trans (1);
  marker.pose.position.z = trans (2);

  marker.pose.orientation.x = quat.x ();
  marker.pose.orientation.y = quat.y ();
  marker.pose.orientation.z = quat.z ();
  marker.pose.orientation.w = quat.w ();*/

  im_ctrl.markers.push_back (marker);

  //  if(!arrows_) {
  // Added For displaying the arrows on Marker Position
  marker_.pose = marker.pose;
  /*marker_.pose.position.x = marker.pose.position.x ;
  marker_.pose.position.y = marker.pose.position.y ;
  marker_.pose.position.z = marker.pose.position.z ;

  marker_.pose.orientation.x = marker.pose.orientation.x ;
  marker_.pose.orientation.y = marker.pose.orientation.y ;
  marker_.pose.orientation.z = marker.pose.orientation.z ;*/
  // end

  delete_contour_marker_ = contour_marker_;
  for (unsigned int i=0; i<delete_contour_marker_.markers.size(); i++)
  {
    delete_contour_marker_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  visualization_msgs::Marker cmarker;
  cmarker.action = visualization_msgs::Marker::ADD;
  cmarker.type = visualization_msgs::Marker::LINE_STRIP;
  cmarker.lifetime = ros::Duration();
  cmarker.header = shape_.header ;
  //cmarker.header.frame_id = "/map";
  cmarker.ns = "contours" ;

  cmarker.scale.x = 0.02;
  cmarker.scale.y = 0.02;
  cmarker.scale.z = 1;

  cmarker.color.r = 0;
  cmarker.color.g = 0;
  cmarker.color.b = 1;
  cmarker.color.a = 1.0;

  cmarker.pose = shape_.pose;

  cob_3d_mapping::Polygon p;
  cob_3d_mapping::fromROSMsg (shape_, p);

  for(unsigned int i=0; i<p.contours_.size(); i++)
  {
    cmarker.id = (p.id_+1)*10+ctr;
    ctr ++ ;
    for(unsigned int j=0; j<p.contours_[i].size(); j++)
    {
      //pcl::PointCloud<pcl::PointXYZ> contour_3d;
      //pcl::TranformPointCloud(p.contours_[i], contour_3d, p.pose_.cast<double>());
      cmarker.points.resize(p.contours_[i].size()+1);
      cmarker.points[j].x = p.contours_[i][j](0);
      cmarker.points[j].y = p.contours_[i][j](1);
      cmarker.points[j].z = 0;//p.contours_[i][j](2);
    }
    cmarker.points[p.contours_[i].size()].x = p.contours_[i][0](0);
    cmarker.points[p.contours_[i].size()].y = p.contours_[i][0](1);
    cmarker.points[p.contours_[i].size()].z = 0;//p.contours[i][0](2);
    contour_marker_.markers.push_back(cmarker);
  }
}

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

void
ShapeMarker::createInteractiveMarker ()
{
  visualization_msgs::InteractiveMarker imarker ;
  // ROS_INFO("\tcreating interactive marker for shape < %d >", shape_.id);

  std::stringstream ss;
  if(!arrows_ && !deleted_) {
    ss.str("");
    ss.clear() ;
    ss << shape_.id ;
    marker_.name = ss.str ();
    marker_.header = shape_.header;
    marker_.header.stamp = ros::Time::now() ;
  }

  else if(arrows_)
  {
    ROS_INFO("Second Marker... ") ;
    ss.str("");
    ss.clear() ;
    ss << "second_marker_" <<shape_.id ;
    imarker_.name = ss.str ();
    imarker_.header = shape_.header;
    imarker_.header.stamp = ros::Time::now() ;
  }
  else if(deleted_)
  {
    ROS_INFO("Deleted Marker... ") ;
    ss.str("");
    ss.clear() ;
    ss << "deleted_marker_" <<shape_.id ;
    deleted_imarker_.name = ss.str ();
    deleted_imarker_.header = shape_.header;
    deleted_imarker_.header.stamp = ros::Time::now() ;
  }
  visualization_msgs::InteractiveMarkerControl im_ctrl_for_second_marker;

  /* create marker */
  if (!deleted_ && !arrows_) {
    ss.str("");
    ss.clear() ;
    ss.str ("");
    im_ctrl.always_visible = true;
    ss << "shape_" << shape_.id << "_control";
    im_ctrl.name = ss.str ();
    im_ctrl.description = "shape_markers";
    im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    createMarker (im_ctrl);
    marker_.controls.push_back(im_ctrl) ;
    im_server_->insert (marker_ );
    im_server_ ->applyChanges() ;
    menu_handler_.apply (*im_server_, marker_.name);
  }
  else if (arrows_)
  {
    createMarker (im_ctrl_for_second_marker);
    imarker_.controls.push_back(im_ctrl_for_second_marker) ;
    im_server_->insert (imarker_ );
    im_server_ ->applyChanges() ;
    //    menu_handler_.apply (*im_server_, imarker_.name);
  }
  else if(deleted_){
    im_ctrl_for_second_marker.always_visible = true;
    im_ctrl_for_second_marker.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    createMarker (im_ctrl_for_second_marker);
    deleted_imarker_.controls.push_back(im_ctrl_for_second_marker) ;
    im_server_->insert (deleted_imarker_ );
    im_server_ ->applyChanges() ;
    menu_handler_.apply (*im_server_, deleted_imarker_.name);
  }
}


void
ShapeMarker::displayNormalCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{

  interactive_markers::MenuHandler::CheckState check_state;

  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);

    displayNormal();
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    hideNormal(1);
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();

}

/*void ShapeMarker::displaySymAxis(){

  ROS_INFO(" displaySymAxis from shape[ %d ]...", shape_.id);

  std::vector<unsigned int>::iterator iter;
  visualization_msgs::InteractiveMarker imarker;
  stringstream ss;

  ss << "symaxis_" << shape_.id;
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
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1;

  //set scale
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  //set pose
  marker.points.resize (2);

  marker.points[0].x = shape_.params[6];
  marker.points[0].y = shape_.params[7];
  marker.points[0].z = shape_.params[8];

  marker.points[1].x = shape_.params[6] - shape_.params[3];
  marker.points[1].y = shape_.params[7] - shape_.params[4];
  marker.points[1].z = shape_.params[8] - shape_.params[5];

  visualization_msgs::InteractiveMarkerControl im_ctrl_n;

  ss << "symaxis_ctrl_" << shape_.id;
  im_ctrl_n.name = ss.str ();
  im_ctrl_n.description = "display_symaxis";

  im_ctrl_n.markers.push_back (marker);
  imarker.controls.push_back (im_ctrl_n);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;


}*/

/*void ShapeMarker::hideSymAxis(int untick){

  stringstream ss;
  std::vector<unsigned int>::iterator iter;

  ss << "symaxis_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  if(untick){
    // updating interacted_shapes_ vector
    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), shape_.id) ;
    if (iter!=interacted_shapes_.end()){
      interacted_shapes_.erase(interacted_shapes_.begin()+(iter-interacted_shapes_.begin())) ;
    }
  }
}*/
/**
 * @brief Display the normal vector of a shape
 */
void ShapeMarker::displayNormal(){

  ROS_INFO(" displayNormalCB from shape[ %d ]...", shape_.id);

  std::vector<unsigned int>::iterator iter;
  visualization_msgs::InteractiveMarker imarker;
  std::stringstream ss;

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
  marker.points[0] = shape_.pose.position;

  marker.points[1].x = shape_.pose.position.x + shape_.params[0];
  marker.points[1].y = shape_.pose.position.y + shape_.params[1];
  marker.points[1].z = shape_.pose.position.z + shape_.params[2];

  visualization_msgs::InteractiveMarkerControl im_ctrl_n;

  ss << "normal_ctrl_" << shape_.id;
  im_ctrl_n.name = ss.str ();
  im_ctrl_n.description = "display_normal";

  //im_ctrl_n.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  im_ctrl_n.markers.push_back (marker);
  imarker.controls.push_back (im_ctrl_n);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;
}

void ShapeMarker::hideNormal(int untick){

  std::stringstream ss;
  std::vector<unsigned int>::iterator iter;

  ss << "normal_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();
  if(!untick){ // when ResetAll is activated
    menu_handler_.setCheckState (2, interactive_markers::MenuHandler::UNCHECKED);//second menu Entry is display Contour
    menu_handler_.reApply (*im_server_);
    im_server_->applyChanges() ;
  }

  if(untick){
    // updating interacted_shapes_ vector
    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), shape_.id) ;
    if (iter!=interacted_shapes_.end()){
      interacted_shapes_.erase(interacted_shapes_.begin()+(iter-interacted_shapes_.begin())) ;
    }
  }
}
//

/*void
ShapeMarker::displayOriginCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  stringstream ss;
  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    displayOrigin();
  }
  if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    hideOrigin(1);
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();


}*/

/*void ShapeMarker::displayOrigin(){

  ROS_INFO(" displayOriginCB from shape[ %d ]...", shape_.id);
  std::vector<unsigned int>::iterator iter;

  stringstream ss;
  ss.clear();
  ss.str("");
  visualization_msgs::InteractiveMarker imarker;
  ss << "origin_" << shape_.id;
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
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1;

  //set scale
  marker.scale.x = 0.04;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;

  //set pose
  marker.pose.position.x = shape_.params[6];
  marker.pose.position.y = shape_.params[7];
  marker.pose.position.z = shape_.params[8];


  visualization_msgs::InteractiveMarkerControl im_ctrl;
  im_ctrl.always_visible = true;
  ss << "origin_ctrl_" << shape_.id;
  im_ctrl.name = ss.str ();
  im_ctrl.markers.push_back (marker);
  imarker.controls.push_back (im_ctrl);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;

}*/

/*void ShapeMarker::hideOrigin(int untick){
  stringstream ss;
  std::vector<unsigned int>::iterator iter;

  ss.clear();
  ss.str("");
  ss << "origin_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  if(untick){
    // updating interacted_shapes_ vector
    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), shape_.id) ;
    if (iter!=interacted_shapes_.end()){
      interacted_shapes_.erase(interacted_shapes_.begin()+(iter-interacted_shapes_.begin())) ;
    }
  }
  //
}*/


/**
 * @brief Feedback callback for Display Centroid menu entry
 *
 * @param feedback feedback from rviz when the Display Centroid menu entry of a shape is changed
 */
void
ShapeMarker::displayCentroidCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  std::stringstream ss;
  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    displayCentroid();
  }
  if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    hideCentroid(1);
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();


}

void ShapeMarker::displayCentroid(){

  ROS_INFO(" displayCentroidCB from shape[ %d ]...", shape_.id);
  std::vector<unsigned int>::iterator iter;

  std::stringstream ss;
  ss.clear();
  ss.str("");
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
  marker.pose.position = shape_.pose.position;


  visualization_msgs::InteractiveMarkerControl im_ctrl;
  im_ctrl.always_visible = true;
  ss << "centroid_ctrl_" << shape_.id;
  im_ctrl.name = ss.str ();
  im_ctrl.markers.push_back (marker);
  imarker.controls.push_back (im_ctrl);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;
}

void ShapeMarker::hideCentroid(int untick){
  std::stringstream ss;
  std::vector<unsigned int>::iterator iter;

  ss.clear();
  ss.str("");
  ss << "centroid_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  if(!untick){ // when ResetAll is activated
    menu_handler_.setCheckState (3, interactive_markers::MenuHandler::UNCHECKED); //third menu Entry is display Contour
    menu_handler_.reApply (*im_server_);
    im_server_->applyChanges() ;
  }

  if(untick){
    // updating interacted_shapes_ vector
    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), shape_.id) ;
    if (iter!=interacted_shapes_.end()){
      interacted_shapes_.erase(interacted_shapes_.begin()+(iter-interacted_shapes_.begin())) ;
    }
  }
}

void ShapeMarker::displayContourCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {

  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    displayContour();
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    hideContour(1);
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();
}


void ShapeMarker::displayContour(){
  ROS_INFO(" displayContourCB from shape[ %d ]...", shape_.id);
  std::vector<unsigned int>::iterator iter ;

  std::stringstream aa;
  std::stringstream ss;
  int ctr = 0;

  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration();
  marker.header = shape_.header ;
  //marker.header.frame_id = "/map";
  marker.ns = "contours" ;

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 1;

  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1.0;

  marker.pose = shape_.pose;

  cob_3d_mapping::Polygon p;
  cob_3d_mapping::fromROSMsg (shape_, p);

  visualization_msgs::InteractiveMarker imarker;
  visualization_msgs::InteractiveMarkerControl im_ctrl_ ;
  for(unsigned int i=0; i<p.contours_.size(); i++)
  {
    marker.id = ctr ;
    ctr ++ ;
    for(unsigned int j=0; j<p.contours_[i].size(); j++)
    {
      //pcl::PointCloud<pcl::PointXYZ> contour_3d;
      //pcl::TranformPointCloud(p.contours_[i], contour_3d, p.pose_.cast<double>());
      marker.points.resize(p.contours_[i].size()+1);
      marker.points[j].x = p.contours_[i][j](0);
      marker.points[j].y = p.contours_[i][j](1);
      marker.points[j].z = 0;//p.contours_[i][j](2);
    }
    marker.points[p.contours_[i].size()].x = p.contours_[i][0](0);
    marker.points[p.contours_[i].size()].y = p.contours_[i][0](1);
    marker.points[p.contours_[i].size()].z = 0;//p.contours[i][0](2);
    im_ctrl_.markers.push_back(marker);

  }

  im_ctrl_.always_visible = true ;
  im_ctrl_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  ss << "contour_" << shape_.id;
  imarker.name = ss.str() ;

  imarker.header  = shape_.header ;
  imarker.controls.push_back(im_ctrl_);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;

}

void ShapeMarker::hideContour(int untick){
  std::stringstream ss;
  std::vector<unsigned int>::iterator iter;

  ss.clear() ;
  ss.str("");
  ss << "contour_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();
  if(!untick){ // when ResetAll is activated
    menu_handler_.setCheckState (4, interactive_markers::MenuHandler::UNCHECKED);  //4th menu Entry is display Contour
    menu_handler_.reApply (*im_server_);
    im_server_->applyChanges() ;
  }

  if(untick){
    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), shape_.id) ;
    if (iter!=interacted_shapes_.end()){
      interacted_shapes_.erase(interacted_shapes_.begin()+(iter-interacted_shapes_.begin())) ;
    }
  }
}

void
ShapeMarker::displaySymAxisCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{

  interactive_markers::MenuHandler::CheckState check_state;

  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    displaySymAxis();
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    hideSymAxis(1);
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();

}

void ShapeMarker::displaySymAxis(){

  ROS_INFO(" displaySymAxis from shape[ %d ]...", shape_.id);

  visualization_msgs::InteractiveMarker imarker;
  std::stringstream ss;

  ss << "symaxis_" << shape_.id;
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
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1;

  //set scale
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  //set pose
  marker.points.resize (2);

  marker.points[0].x = shape_.params[6];
  marker.points[0].y = shape_.params[7];
  marker.points[0].z = shape_.params[8];

  marker.points[1].x = shape_.params[6] - shape_.params[3];
  marker.points[1].y = shape_.params[7] - shape_.params[4];
  marker.points[1].z = shape_.params[8] - shape_.params[5];

  visualization_msgs::InteractiveMarkerControl im_ctrl_n;

  ss << "symaxis_ctrl_" << shape_.id;
  im_ctrl_n.name = ss.str ();
  im_ctrl_n.description = "display_symaxis";

  im_ctrl_n.markers.push_back (marker);
  imarker.controls.push_back (im_ctrl_n);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;


}

void ShapeMarker::hideSymAxis(int untick){

  std::stringstream ss;
  std::vector<unsigned int>::iterator iter;

  ss << "symaxis_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  if(untick){
    // updating interacted_shapes_ vector
    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), shape_.id) ;
    if (iter!=interacted_shapes_.end()){
      interacted_shapes_.erase(interacted_shapes_.begin()+(iter-interacted_shapes_.begin())) ;
    }
  }
  if(!untick){ // when ResetAll is activated
    menu_handler_.setCheckState (7, interactive_markers::MenuHandler::UNCHECKED);//second menu Entry is display Contour
    menu_handler_.reApply (*im_server_);
    im_server_->applyChanges() ;
  }
}

void
ShapeMarker::displayOriginCB (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  std::stringstream ss;
  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    displayOrigin();
  }
  if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    hideOrigin(1);
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();


}

void ShapeMarker::displayOrigin(){

  ROS_INFO(" displayOriginCB from shape[ %d ]...", shape_.id);
  std::vector<unsigned int>::iterator iter;

  std::stringstream ss;
  ss.clear();
  ss.str("");
  visualization_msgs::InteractiveMarker imarker;
  ss << "origin_" << shape_.id;
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
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1;

  //set scale
  marker.scale.x = 0.04;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;

  //set pose
  marker.pose.position.x = shape_.params[6];
  marker.pose.position.y = shape_.params[7];
  marker.pose.position.z = shape_.params[8];


  visualization_msgs::InteractiveMarkerControl im_ctrl;
  im_ctrl.always_visible = true;
  ss << "origin_ctrl_" << shape_.id;
  im_ctrl.name = ss.str ();
  im_ctrl.markers.push_back (marker);
  imarker.controls.push_back (im_ctrl);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;

}

void ShapeMarker::hideOrigin(int untick){
  std::stringstream ss;
  std::vector<unsigned int>::iterator iter;

  ss.clear();
  ss.str("");
  ss << "origin_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  if(untick){
    // updating interacted_shapes_ vector
    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), shape_.id) ;
    if (iter!=interacted_shapes_.end()){
      interacted_shapes_.erase(interacted_shapes_.begin()+(iter-interacted_shapes_.begin())) ;
    }
  }
  if(!untick){ // when ResetAll is activated
    menu_handler_.setCheckState (8, interactive_markers::MenuHandler::UNCHECKED);//second menu Entry is display Contour
    menu_handler_.reApply (*im_server_);
    im_server_->applyChanges() ;
  }
  //
}

void ShapeMarker::displayIDCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{

  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);
  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    //ROS_INFO(" entry state changed ");
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);
    displayID();
  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);
    hideID(1);
  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges ();
}


void ShapeMarker::displayID()
{
  ROS_INFO(" displayID from shape[ %d ]...", shape_.id);
  std::vector<unsigned int>::iterator iter ;

  std::stringstream aa;
  std::stringstream ss;
  int ctr = 0;

  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.lifetime = ros::Duration();
  marker.header = shape_.header ;
  //marker.header.frame_id = "/map";
  marker.ns = "contours" ;

  //set pose
  marker.pose = shape_.pose;
  /*marker.pose.position.x = shape_.centroid.x;
  marker.pose.position.y = shape_.centroid.y;
  marker.pose.position.z = shape_.centroid.z;*/

  marker.scale.z = 0.1;

  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1.0;


  visualization_msgs::InteractiveMarker imarker;
  visualization_msgs::InteractiveMarkerControl im_ctrl_ ;

  im_ctrl_.markers.push_back(marker);

  im_ctrl_.always_visible = true ;
  im_ctrl_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  ss << "id_" << shape_.id;
  imarker.name = ss.str() ;

  imarker.header  = shape_.header ;
  imarker.controls.push_back(im_ctrl_);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;

}

void ShapeMarker::hideID(int untick)
{
  std::stringstream ss;
  std::vector<unsigned int>::iterator iter;

  ss.clear() ;
  ss.str("");
  ss << "id_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();
  if(!untick){ // when ResetAll is activated
    menu_handler_.setCheckState (4, interactive_markers::MenuHandler::UNCHECKED);  //4th menu Entry is display Contour
    menu_handler_.reApply (*im_server_);
    im_server_->applyChanges() ;
  }

  if(untick){
    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), shape_.id) ;
    if (iter!=interacted_shapes_.end()){
      interacted_shapes_.erase(interacted_shapes_.begin()+(iter-interacted_shapes_.begin())) ;
    }
  }
}

