/*
 * shape_marker.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: goa-sn
 */

#include <cob_3d_visualization/shape_marker.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

void ShapeMarker::getShape (cob_3d_mapping_msgs::Shape& shape) {
  shape_ = shape ;
}
unsigned int ShapeMarker::getID(){
  return id_;
}

void ShapeMarker::getMarker (int id) {
  stringstream aa;
  aa.clear();
  aa.str("");
  aa << id ;
  im_server_->get(aa.str(),marker_);
  im_server_->insert(marker_);
  im_server_->applyChanges ();
}
void ShapeMarker::deleteMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {

  stringstream ss;
  ss << shape_.id ;// ctr_ ;
  deleted_markers_indices_.push_back(shape_.id) ;
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.name = ss.str() ;
  std::cout << "Marker" << interactiveMarker.name << " deleted..."<< std::endl ;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();
}



void ShapeMarker::enableMovement (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  interactive_markers::MenuHandler::CheckState check_state;
  menu_handler_.getCheckState (feedback->menu_entry_id, check_state);

  shape_.header = marker_.header ;
  shape_.header.frame_id = "/map" ;

  if (check_state == interactive_markers::MenuHandler::UNCHECKED)
  {
    displayArrows();
    //    im_server_->setCallback(marker_.name,boost::bind (&ShapeMarker::setShapePosition, this, _1)
    //                              ,visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED);

  }
  else if (check_state == interactive_markers::MenuHandler::CHECKED)
  {
    hideArrows(1);
    menu_handler_.setCheckState (feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED);

  }
  menu_handler_.reApply (*im_server_);
  im_server_->applyChanges();
}


void ShapeMarker::displayArrows()
{
  //  ros::NodeHandle nh;
  //  nh.subscribe("geometry_map/map/feedback",1,&ShapeMarker::setShapePosition,this);

  //  arrows_ = true;

  visualization_msgs::InteractiveMarkerControl im_ctrl;
  stringstream ss;
  ss.str("");
  ss.clear();

  marker_.header  = shape_.header ;
  marker_.header.frame_id = "/map" ;


  //  if (flag == 1) {
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

  // Save the ids of shapes that have been moved
  //    movedShapesIndexes.push_back(shape_.id) ;

  moved_shapes_indices_.push_back(shape_.id) ;

  im_server_->insert (marker_);
  im_server_->applyChanges() ;


}

//  else if (flag == 0)
void ShapeMarker::hideArrows(int untick)
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

  if (untick){
    for (unsigned int i=0;i< moved_shapes_indices_.size();i++){
      if (moved_shapes_indices_.at(i) == shape_.id){
        moved_shapes_indices_.erase(moved_shapes_indices_.begin()+i) ;
      }
    }
  }
  im_server_->insert (marker_);
  im_server_->applyChanges() ;
}


void ShapeMarker::resetMarker(){   //bool reset_marker,visualization_msgs::InteractiveMarker& imarker) {

  stringstream aa;
  stringstream ss;

  hideNormal(0);
  hideCentroid(0);
  hideContour(0);

  //  interacted_shapes_.pop_back() ;

}
void
ShapeMarker::createShapeMenu ()
{
  //  ROS_INFO(" creating menu .....");

  interactive_markers::MenuHandler::EntryHandle eh_1, eh_2, eh_3, eh_4, eh_5, eh_6;

  eh_1 = menu_handler_.insert ("Menu");
  eh_2 = menu_handler_.insert (eh_1, "Display Normal",boost::bind (&ShapeMarker::displayNormalCB, this, _1));
  eh_3 = menu_handler_.insert (eh_1, "Display Centroid",boost::bind (&ShapeMarker::displayCentroidCB, this, _1));
  eh_4 = menu_handler_.insert (eh_1, "Display Contour",boost::bind (&ShapeMarker::displayContourCB, this, _1));
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

}
void
ShapeMarker::createMarker (list<TPPLPoly>& triangle_list, visualization_msgs::InteractiveMarkerControl& im_ctrl)
{
  //ROS_INFO(" creating markers for this shape.....");
  TPPLPoint pt;
  for (std::list<TPPLPoly>::iterator it = triangle_list.begin (); it != triangle_list.end (); it++)
  {
    marker.id = shape_.id;

    marker.header = shape_.header;
    marker.header.stamp = ros::Time::now() ;

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
  //  ROS_INFO("\tcreating interactive marker for shape < %d >", shape_.id);

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
  marker_.header.stamp = ros::Time::now() ;

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

void ShapeMarker::displayNormal(){

  ROS_INFO(" displayNormalCB from shape[ %d ]...", shape_.id);

  std::vector<unsigned int>::iterator iter;
  visualization_msgs::InteractiveMarker imarker;
  stringstream ss;

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

  interacted_shapes_.push_back(shape_.id) ;
  //  if (interacted_shapes_.empty()){
  //    interacted_shapes_.push_back(shape_.id) ;
  //  }
  //  else {
  //    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), id_) ;
  //    if (iter == interacted_shapes_.end()) {
  //      interacted_shapes_.push_back(shape_.id) ;
  //    }
  //  }
  //
  //  for (unsigned int i=0;i<interacted_shapes_.size();i++){
  //    std::cout << interacted_shapes_.at(i) << "\t" ;
  //  }
  //  std::cout << "\n" ;


}

void ShapeMarker::hideNormal(int untick){
  stringstream ss;
  ss << "normal_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  if(untick){
    // updating interacted_shapes_ vector
    for (unsigned int i=0;i<interacted_shapes_.size();i++){
      if (interacted_shapes_.at(i) == shape_.id){
        interacted_shapes_.erase(interacted_shapes_.begin()+i) ;
        break ;
      }
    }
  }
  //
  //  for (unsigned int i=0;i<interacted_shapes_.size();i++){
  //    std::cout << interacted_shapes_.at(i) << "\t" ;
  //  }
  //  std::cout << "\n" ;
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
  stringstream ss;
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

  stringstream ss;
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

  interacted_shapes_.push_back(shape_.id) ;
  //  if (interacted_shapes_.empty()){
  //    interacted_shapes_.push_back(id_) ;
  //    std::cout << "empty!!!!!" << "\n" ;
  //
  //  }
  //  else {
  //    std::cout << "shape_.id" << shape_.id << "\n" ;
  //    for (unsigned int i=0;i<interacted_shapes_.size() ;i++){
  //      if (interacted_shapes_.at(i) != shape_.id){
  //        std::cout << "interacted_shapes_.at" << "(" << i <<")"<< " = "<<  interacted_shapes_.at(i)<< "\n" ;
  //        interacted_shapes_.push_back(shape_.id) ;
  //        break ;
  //      }
  //    }
  //  }
  //  if (interacted_shapes_.empty()){
  //    interacted_shapes_.push_back(shape_.id) ;
  //  }
  //  else {
  //    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), id_) ;
  //    if (iter == interacted_shapes_.end()) {
  //      interacted_shapes_.push_back(shape_.id) ;
  //    }
  //  }
  //
  //  for (unsigned int i=0;i<interacted_shapes_.size();i++){
  //    std::cout << interacted_shapes_.at(i) << "\t" ;
  //  }
  //  std::cout << "\n" ;
}

void ShapeMarker::hideCentroid(int untick){
  stringstream ss;
  ss.clear();
  ss.str("");
  ss << "centroid_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  if(untick){
    // updating interacted_shapes_ vector
    for (unsigned int i=0;i<interacted_shapes_.size();i++){
      if (interacted_shapes_.at(i) == shape_.id){
        interacted_shapes_.erase(interacted_shapes_.begin()+i) ;
        break;
      }
    }
  }
  //
  //  for (unsigned int i=0;i<interacted_shapes_.size();i++){
  //    std::cout << interacted_shapes_.at(i) << "\t" ;
  //  }
  //  std::cout << "\n" ;

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

  visualization_msgs::InteractiveMarker imarker;
  for(unsigned int i=0; i<p.contours.size(); i++)
  {
    marker.id = ctr ;
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
  im_ctrl_.always_visible = true ;
  im_ctrl_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  ss << "contour_" << shape_.id;
  imarker.name = ss.str() ;

  imarker.header  = shape_.header ;
  im_ctrl_.markers.push_back(marker);
  imarker.controls.push_back(im_ctrl_);
  im_server_->insert (imarker);

  interacted_shapes_.push_back(shape_.id) ;

  //  if (interacted_shapes_.empty()){
  //    interacted_shapes_.push_back(shape_.id) ;
  //  }
  //  else {
  //    iter = find (interacted_shapes_.begin(), interacted_shapes_.end(), id_) ;
  //    if (iter == interacted_shapes_.end()) {
  //      interacted_shapes_.push_back(shape_.id) ;
  //    }
  //  }
  //  else {
  //    std::cout <<"shape_.id" << shape_.id << "\n" ;
  //    for (unsigned int i=0;i<interacted_shapes_.size() ;i++){
  //      if (interacted_shapes_.at(i) != shape_.id){
  //        std::cout << "interacted_shapes_.at" << "(" << i <<")"<< " = "<<  interacted_shapes_.at(i)<< "\n" ;
  //        interacted_shapes_.push_back(shape_.id) ;
  //        break ;
  //      }
  //    }
  //  }
  //  for (unsigned int i=0;i<interacted_shapes_.size();i++){
  //    std::cout << interacted_shapes_.at(i) << "\t" ;
  //  }
  //  std::cout << "\n" ;
}

void ShapeMarker::hideContour(int untick){
  stringstream ss;
  ss.clear() ;
  ss.str("");
  ss << "contour_" << shape_.id;
  im_server_->erase(ss.str());
  im_server_->applyChanges ();

  if(untick){
    //   updating interacted_shapes_ vector
    for (unsigned int i=0;i<interacted_shapes_.size();i++){
      if (interacted_shapes_.at(i) == shape_.id){
        interacted_shapes_.erase(interacted_shapes_.begin()+i) ;
        break;
      }
    }
  }

}
//void ShapeMarker::setShapePosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
//{
//  if (arrows_){
//    ROS_INFO("In setShapePosition...") ;
//
//    cob_3d_mapping_msgs::ShapeArray map_msg;
//    map_msg.header.frame_id="/map";
//    map_msg.header.stamp = ros::Time::now();
//
//    //  int shape_id;
//    //  Eigen::Affine3f trans;
//
//
//
//    //  stringstream name(feedback->marker_name);
//
//
//    Eigen::Quaternionf quat;
//    Eigen::Matrix3f rotationMat;
//    Eigen::MatrixXf rotationMatInit;
//    Eigen::Vector3f vec;
//    Eigen::Vector3f vecNew;
//    Eigen::Vector3f newCentroid;
//    Eigen::Matrix4f transSecondStep;
//    cob_3d_mapping_msgs::ShapeArray modified_shapes;
//    //  Eigen::Affine3f transformInit ;
//
//    //    if (feedback->marker_name != "Text"){
//    //    name >> shape_id ;
//
//    cob_3d_mapping::Polygon p;
//    cob_3d_mapping::fromROSMsg (shape_, p);
//
//
//    if (feedback->menu_entry_id == 5){
//      //     if (arrows_){
//      //       if (feedback->event_type == 2){
//
//      Eigen::Vector3f oldCentroid ;
//      Eigen::Matrix4f transInit;
//
//      quatInit.x() = (feedback->pose.orientation.x) ;           //normalized
//      quatInit.y() = (feedback->pose.orientation.y) ;
//      quatInit.z() = (feedback->pose.orientation.z) ;
//      quatInit.w() = (feedback->pose.orientation.w) ;
//
//      oldCentroid (0) = feedback->pose.position.x ;
//      oldCentroid (1) = feedback->pose.position.y ;
//      oldCentroid (2) = feedback->pose.position.z ;
//
//      //    quatInit.normalize() ;
//
//      rotationMatInit = quatInit.toRotationMatrix() ;
//
//      transInit.block(0,0,3,3) << rotationMatInit ;
//      transInit.col(3).head(3) << oldCentroid(0) , oldCentroid(1), oldCentroid(2) ;
//      transInit.row(3) << 0,0,0,1 ;
//
//      transInitInv = transInit.inverse() ;
//      Eigen::Affine3f affineInitFinal(transInitInv) ;
//      affineInit = affineInitFinal ;
//
//      //       }
//    }
//    if (feedback->event_type == 1){
//
//      std::cout << "transInit : " << "\n"    << affineInit.matrix() << "\n" ;
//
//      quat.x() = feedback->pose.orientation.x ;           //normalized
//      quat.y() = feedback->pose.orientation.y ;
//      quat.z() = feedback->pose.orientation.z ;
//      quat.w() = feedback->pose.orientation.w ;
//
//      quat.normalize() ;
//
//      rotationMat = quat.toRotationMatrix() ;
//
//      vec << shape_.params[0],                   //normalized
//          shape_.params[1],
//          shape_.params[2];
//
//      shape_.centroid.x = feedback->pose.position.x ;
//      shape_.centroid.y = feedback->pose.position.y ;
//      shape_.centroid.z = feedback->pose.position.z ;
//
//      newCentroid << shape_.centroid.x ,
//          shape_.centroid.y ,
//          shape_.centroid.z ;
//
//
//      transSecondStep.block(0,0,3,3) << rotationMat ;
//      transSecondStep.col(3).head(3) << newCentroid(0) , newCentroid(1), newCentroid(2) ;
//      transSecondStep.row(3) << 0,0,0,1 ;
//
//      Eigen::Affine3f affineSecondStep(transSecondStep) ;
//
//      std::cout << "transfrom : " << "\n"    << affineSecondStep.matrix() << "\n" ;
//
//      //        std::cout << "affineInit :" << "\n"    << affineInit.matrix() << "\n" ;
//
//      Eigen::Affine3f affineFinal(affineSecondStep*affineInit) ;
//      Eigen::Matrix4f matFinal = (transSecondStep*transInitInv);       //transInitInv) ;
//
//      vecNew    = (matFinal.block(0,0,3,3))* vec;
//      //      newCentroid  = transFinal *OldCentroid ;
//
//
//      shape_.centroid.x = newCentroid(0) ;
//      shape_.centroid.y = newCentroid(1) ;
//      shape_.centroid.z = newCentroid(2) ;
//
//
//      shape_.params[0] = vecNew(0) ;
//      shape_.params[1] = vecNew(1) ;
//      shape_.params[2] = vecNew(2) ;
//
//
//      std::cout << "transfromFinal : " << "\n"    << affineFinal.matrix() << "\n" ;
//
//      pcl::PointCloud<pcl::PointXYZ> pc;
//      pcl::PointXYZ pt;
//      sensor_msgs::PointCloud2 pc2;
//
//      for(unsigned int j=0; j<p.contours.size(); j++)
//      {
//        for(unsigned int k=0; k<p.contours[j].size(); k++)
//        {
//          p.contours[j][k] = affineFinal * p.contours[j][k];
//          pt.x = p.contours[j][k][0] ;
//          pt.y = p.contours[j][k][1] ;
//          pt.z = p.contours[j][k][2] ;
//          pc.push_back(pt) ;
//        }
//      }
//
//      pcl::toROSMsg (pc, pc2);
//      shape_.points.clear() ;
//      shape_.points.push_back (pc2);
//
//      // uncomment when using test_shape_array
//
//      //      for(unsigned int i=0;i<sha.shapes.size();i++){
//      //        map_msg.header = sha.shapes.at(i).header ;
//      //        map_msg.shapes.push_back(sha.shapes.at(i)) ;
//      //      }
//      //      shape_pub_.publish(map_msg);
//
//      // end uncomment
//      //      quatInit.Identity() ;
//      //      oldCentroid.Identity() ;
//      //      transInit.Identity();
//      //      affineInit.Identity();
//      //      transInitInv.Identity();
//
//      modified_shapes_.shapes.push_back(shape_);
//      std::cout << "modified_shapes_.shapes.size(): " << modified_shapes_.shapes.size() << "\n";
//
//      //    modified_shapes_.shapes.push_back(shape_);
//      //      std::cout << "req.InMap.shapes.size()" << req.InMap.shapes.size() << "\n";
//      //      arrows_ = false;
//    }
//    if (feedback->event_type == 5){
//      //      modified_shapes_.shapes.push_back(shape_);
//      //      std::cout << "modified_shapes_.shapes.size(): " << modified_shapes_.shapes.size() << "\n";
//      arrows_ = false;
//    }
//  }
//}



