/*
 * move_to_table_node.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: goa-sn
 */
#include <cob_3d_mapping_semantics/move_to_table_node.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include <cob_3d_mapping_common/polygon.h>
/**
 * @brief transforms a point to table coordinate system
 *
 * @param table table msg
 * @param pose the pose which needs to be transformed ro table coordinate system
 *
 * @return the transformed point
 */
geometry_msgs::Pose MoveToTableNode::transformToTableCoordinateSystem(tabletop_object_detector::Table &table,geometry_msgs::Pose &Pose)
{
  Eigen::Matrix4f transformationMat ;
  Eigen::Quaternionf quat ;
  Eigen::Vector4f poseInTableCoordinateSys ;
  Eigen::Vector4f poseInOwnCoordinateSys ;
  Eigen::Affine3f poseInOwnCoordinateSysAffine ;

  geometry_msgs::Pose translatedPose;

  poseInOwnCoordinateSys << Pose.position.x,
      Pose.position.y,
      Pose.position.z,
      1;

  quat.x() = table.pose.pose.orientation.x ;
  quat.y() = table.pose.pose.orientation.y ;
  quat.z() = table.pose.pose.orientation.z ;
  quat.w() = table.pose.pose.orientation.w ;

  quat.normalize() ;

  transformationMat.block(0,0,3,3) = quat.toRotationMatrix();
  transformationMat.col(3).head(3) <<  table.pose.pose.position.x,table.pose.pose.position.y,table.pose.pose.position.z ;
  transformationMat.row(3) << 0,0,0,1 ;

  transformToTableCoordinateSys_ = transformationMat ;

  poseInTableCoordinateSys = transformationMat*poseInOwnCoordinateSys ;

  translatedPose.position.x = poseInTableCoordinateSys(0) ;
  translatedPose.position.y = poseInTableCoordinateSys(1) ;
  translatedPose.position.z = poseInTableCoordinateSys(2) ;

  return translatedPose ;

}
/**
 * @brief finds whether there is an intersection between the line through the robot pose and table centroid and the boundies of the table
 *
 * @return true if there exists an Intersection
 */
bool MoveToTableNode::doIntersect(float line){

  bool intersection(false) ;
  geometry_msgs::Pose point3,point4 ;


  if (line == table_.y_max) {
    point3.position.x = table_.x_max ;
    point3.position.y = table_.y_max ;

    point4.position.x = table_.x_min ;
    point4.position.y = table_.y_max ;
  }

  if (line == table_.x_max) {
    point3.position.x = table_.x_max ;
    point3.position.y = table_.y_max ;

    point4.position.x = table_.x_max ;
    point4.position.y = table_.y_min ;
  }

  if (line == table_.y_min) {
    point3.position.x = table_.x_min ;
    point3.position.y = table_.y_min ;

    point4.position.x = table_.x_max ;
    point4.position.y = table_.y_min ;
  }

  if (line == table_.x_min) {
    point3.position.x = table_.x_min ;
    point3.position.y = table_.y_min ;

    point4.position.x = table_.x_min ;
    point4.position.y = table_.y_max ;
  }
  Eigen::Matrix3f mat1 ;
  mat1 << robotPoseInTableCoordinateSys_.position.x,robotPoseInTableCoordinateSys_.position.y,1,
      0,0,1,
      point3.position.x,point3.position.y,1 ;

  Eigen::Matrix3f mat2 ;
  mat2 << robotPoseInTableCoordinateSys_.position.x,robotPoseInTableCoordinateSys_.position.y,1,
      0,0,1,
      point4.position.x,point4.position.y,1 ;

  Eigen::Matrix3f mat3 ;
  mat3 << point3.position.x,point3.position.y,1,
      point4.position.x,point4.position.y,1,
      0,0,1;

  Eigen::Matrix3f mat4 ;
  mat4 << point3.position.x,point3.position.y,1,
      point4.position.x,point4.position.y,1,
      robotPoseInTableCoordinateSys_.position.x,robotPoseInTableCoordinateSys_.position.y,1;

  if (mat1.determinant()*mat2.determinant() <0 &&
      mat3.determinant()*mat4.determinant() <0) {

    intersection = true ;
  }
  return intersection ;
}
/**
 * @brief finds the intersection between the line through the robot pose and table centroid and the boundies of the table
 * @return the intersection point
 */
geometry_msgs::Pose MoveToTableNode::findIntersectionPoint(){

  geometry_msgs::Pose IntersectionPoint ;

  bool xMax = doIntersect(table_.x_max) ;   // check if there is an intersection between the line passing through robot pose and table centroid and x=table.x_max
  bool yMax = doIntersect(table_.y_max) ;   // doing the same for the line passing through robot pose and table centroid and y=table.y_max
  bool xMin = doIntersect(table_.x_min) ;   // doing the same for the line passing through robot pose and table centroid and x=table.x_min
  bool yMin = doIntersect(table_.y_min) ;   // doing the same for the line passing through robot pose and table centroid and x=table.x_max

  if (xMax) {
    ROS_INFO("intersection with x = x_max ...");
    IntersectionPoint.position.x = table_.x_max ;
    IntersectionPoint.position.y = (table_.x_max/robotPoseInTableCoordinateSys_.position.x)*robotPoseInTableCoordinateSys_.position.y ;
  }

  else if (yMax) {
    ROS_INFO("intersection with y = y_max ...");
    IntersectionPoint.position.y = table_.y_max ;
    IntersectionPoint.position.x = (table_.y_max/robotPoseInTableCoordinateSys_.position.y)*robotPoseInTableCoordinateSys_.position.x ;
  }

  else if (xMin) {
    ROS_INFO("intersection with x = x_min ...");
    IntersectionPoint.position.x = table_.x_min ;
    IntersectionPoint.position.y = (table_.x_min/robotPoseInTableCoordinateSys_.position.x)*robotPoseInTableCoordinateSys_.position.y ;
  }
  else if (yMin) {
    ROS_INFO("intersection with y = y_min ...");
    IntersectionPoint.position.y = table_.y_min ;
    IntersectionPoint.position.x = (table_.y_min/robotPoseInTableCoordinateSys_.position.y)*robotPoseInTableCoordinateSys_.position.x ;
  }
  else {
    ROS_INFO("no intersection...") ;
  }

  return IntersectionPoint ;

}
/**
 * @brief finds a safe position in the vicinity of the table as the target
 *
 * @return the position of the target point
 */
geometry_msgs::Pose MoveToTableNode::findSafeTargetPoint(){

  geometry_msgs::Pose intersectionPoint = findIntersectionPoint() ;
  geometry_msgs::Pose finalTarget ;

  // solve the quadratic equation to find the point which its distance to the intersection point equals to safeDist_

  float a((robotPoseInTableCoordinateSys_.position.x*robotPoseInTableCoordinateSys_.position.x)+
      (robotPoseInTableCoordinateSys_.position.y*robotPoseInTableCoordinateSys_.position.y));

  float b ((-2*(intersectionPoint.position.x *robotPoseInTableCoordinateSys_.position.x))-
      (2*(intersectionPoint.position.y *robotPoseInTableCoordinateSys_.position.y)));

  float c((intersectionPoint.position.x*intersectionPoint.position.x)+(intersectionPoint.position.y*intersectionPoint.position.y)
      -(safe_dist_*safe_dist_));

  float discriminant((b*b)-4*a*c) ;
  float t1 = (-b + sqrt(discriminant))/(2*a) ;
  float t2 = (-b - sqrt(discriminant))/(2*a) ;
  float t ;
  if (t1>= 0 && t1 <=1) {
    t = t1 ;
    finalTarget.position.x = t * robotPoseInTableCoordinateSys_.position.x ;
    finalTarget.position.y = t * robotPoseInTableCoordinateSys_.position.y ;
  }
  else if (t2>= 0 && t2 <=1) {
    t = t2;
    finalTarget.position.x = t * robotPoseInTableCoordinateSys_.position.x ;
    finalTarget.position.y = t * robotPoseInTableCoordinateSys_.position.y ;
  }

  return finalTarget ;

}
/**
 * @brief adds a marker for showing the final target
 * @return nothing
 */
void MoveToTableNode::addMarkerForFinalPose(geometry_msgs::Pose finalPose) {

  std::stringstream ss;
  visualization_msgs::InteractiveMarker imarker;
  ss.str("");
  ss.clear();
  ss << "target";

  imarker.name = ss.str();
  //  imarker.header = shape_.header;

  imarker.header.frame_id = "/map" ;

  visualization_msgs::Marker marker;
  //  marker.header = shape_.header;
  marker.header.frame_id = "/map" ;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration ();

  //set color
  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1;

  //set scale
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  //set pose
  marker.pose.position.x = finalPose.position.x;
  marker.pose.position.y = finalPose.position.y;
  marker.pose.position.z = finalPose.position.z;


  visualization_msgs::InteractiveMarkerControl im_ctrl;

  im_ctrl.always_visible = true;
  ss << "target_ctrl_" ;
  im_ctrl.name = ss.str ();
  im_ctrl.markers.push_back (marker);
  imarker.controls.push_back (im_ctrl);

  table_im_server_->insert (imarker);
  table_im_server_->applyChanges() ;
}
/**
 * @brief service callback for MoveToTable service
 * @param req request  to move to table
 * @param res empty response
 *
 * @return nothing
 */
bool MoveToTableNode::moveToTableService (cob_3d_mapping_msgs::MoveToTable::Request &req,
    cob_3d_mapping_msgs::MoveToTable::Response &res)
{
  ROS_INFO("calling move_to_table Service ...") ;
  Eigen::Vector4f vec,vecFinal;
  geometry_msgs::Pose finalTargetInMapCoordinateSys;

  geometry_msgs::PoseStamped finalPose ;


  // get robot position
  tf::TransformListener listener;
  tf::StampedTransform transform ;

  listener.waitForTransform("/base_link", "/map", ros::Time::now(), ros::Duration(3.0));  //The listener needs to get the information first before it can transform.
  listener.lookupTransform("/base_link","/map",ros::Time(0), transform);

  robotPose_.position.x = transform.getOrigin().x() ;
  robotPose_.position.y = transform.getOrigin().y() ;
  robotPose_.position.z = transform.getOrigin().z() ;

  // test
  //  robotPose_.position.x = 1 ;
  //  robotPose_.position.y = 1 ;
  //  robotPose_.position.z = 0 ;

  cob_3d_mapping::Polygon p;
  fromROSMsg(req.targetTable, p);
  Eigen::Affine3f pose;
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  p.computePoseAndBoundingBox(pose,min_pt, max_pt);
  table_.pose.pose.position.x = pose.translation()(0); //poly_ptr->centroid[0];
  table_.pose.pose.position.y = pose.translation()(1) ;//poly_ptr->centroid[1];
  table_.pose.pose.position.z = pose.translation()(2) ;//poly_ptr->centroid[2];
  Eigen::Quaternionf quat(pose.rotation());

  table_.pose.pose.orientation.x = quat.x();
  table_.pose.pose.orientation.y = quat.y();
  table_.pose.pose.orientation.z = quat.z();
  table_.pose.pose.orientation.w = quat.w();
  table_.x_min = min_pt(0);
  table_.x_max = max_pt(0);
  table_.y_min = min_pt(1);
  table_.y_max = max_pt(1);

  robotPoseInTableCoordinateSys_ = transformToTableCoordinateSystem(table_,robotPose_) ;

  ROS_WARN("robotPoseInTableCoordinateSys: x=%f , y= %f , z= %f",robotPoseInTableCoordinateSys_.position.x
      ,robotPoseInTableCoordinateSys_.position.y
      ,robotPoseInTableCoordinateSys_.position.z);




  geometry_msgs::Pose targetPointInTableCoordinateSys = findSafeTargetPoint() ;
  //  targetPointInTableCoordinateSys.position.z = req.tableCentroid.position.z ;

  vec << targetPointInTableCoordinateSys.position.x ,
      targetPointInTableCoordinateSys.position.y ,
      targetPointInTableCoordinateSys.position.z ,
      1;
  vecFinal << transformToTableCoordinateSys_.inverse() * vec ;

  finalTargetInMapCoordinateSys.position.x = vecFinal (0) ;
  finalTargetInMapCoordinateSys.position.y = vecFinal (1) ;
  finalTargetInMapCoordinateSys.position.z = vecFinal (2) ;
  ROS_WARN("Final Target point: x=%f , y= %f , z= %f",finalTargetInMapCoordinateSys.position.x
      ,finalTargetInMapCoordinateSys.position.y
      ,finalTargetInMapCoordinateSys.position.z);

  addMarkerForFinalPose (finalTargetInMapCoordinateSys) ;
  finalPose.header.frame_id = "/map" ;
  //  res.goalPoint.header.frame_id = "/map" ;

  finalPose.pose.position.x = vecFinal (0) ;
  finalPose.pose.position.y = vecFinal (1) ;
  finalPose.pose.position.z = 0;//vecFinal (2) ;

  finalPose.pose.orientation.x = 0;
  finalPose.pose.orientation.y = 0;
  finalPose.pose.orientation.z = 0;
  finalPose.pose.orientation.w = 1;

  navigation_goal_pub_.publish(finalPose) ;
  return true;
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "move_to_table_node");
  ROS_INFO("move_to_table node started....");
  MoveToTableNode mtt;


  while (ros::ok()){
    ros::spin();
  }
}
