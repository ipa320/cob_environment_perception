/*
 * move_to_table_node.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: goa-sn
 */
#include <cob_3d_mapping_semantics/move_to_table_node.h>

geometry_msgs::Pose MoveToTableNode::calculateNavGoal(tabletop_object_detector::Table &table,geometry_msgs::Pose &tableCentroid
    ,geometry_msgs::Pose &robPose) {

  geometry_msgs::Pose NavGoal;
  return NavGoal ;
}

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

  transformationMat.block(0,0,3,3) = quat.toRotationMatrix();
  transformationMat.col(3).head(3) <<  table.pose.pose.position.x,table.pose.pose.position.y,table.pose.pose.position.z ;
  transformationMat.row(3) << 0,0,0,1 ;

  transformToTableCoordinateSys_ = transformationMat ;

  //  Eigen::Affine3f transformationAffine(transformationMat) ;

  poseInTableCoordinateSys = transformationMat*poseInOwnCoordinateSys ;

  translatedPose.position.x = poseInTableCoordinateSys(0) ;
  translatedPose.position.y = poseInTableCoordinateSys(1) ;
  translatedPose.position.z = poseInTableCoordinateSys(2) ;

  return translatedPose ;

}
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

  ROS_WARN("determinant is : %f",mat1.determinant()*mat2.determinant()) ;
  ROS_WARN("determinant is : %f",mat3.determinant()*mat4.determinant()) ;
  if (mat1.determinant()*mat2.determinant() <0 &&
      mat3.determinant()*mat4.determinant() <0) {

    intersection = true ;
  }
  return intersection ;
}

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

geometry_msgs::Pose MoveToTableNode::findSafeTargetPoint(){

  geometry_msgs::Pose intersectionPoint = findIntersectionPoint() ;
  geometry_msgs::Pose finalTarget ;
  // solve the quadratic equation to find the point which its distance to the intersection point is safeDist_
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





bool MoveToTableNode::moveToTableService (cob_3d_mapping_msgs::MoveToTable::Request &req,
    cob_3d_mapping_msgs::MoveToTable::Response &res)
{
  ROS_INFO("calling move_to_table Service ...") ;
  Eigen::Vector4f vec,vecFinal;
  geometry_msgs::Pose finalTargetInMapCoordinateSys;

  // test
  robotPoseInTableCoordinateSys_.position.x =  2.0;
  robotPoseInTableCoordinateSys_.position.y = -1.0;
  robotPoseInTableCoordinateSys_.position.z = (float)req.tableCentroid.position.z ;
  table_ = req.targetTable ;

  geometry_msgs::Pose targetPointInTableCoordinateSys = findSafeTargetPoint() ;
  targetPointInTableCoordinateSys.position.z = req.tableCentroid.position.z ;

  vec << targetPointInTableCoordinateSys.position.x ,
         targetPointInTableCoordinateSys.position.y ,
         targetPointInTableCoordinateSys.position.z ,
         1;
  vecFinal << transformToTableCoordinateSys_.inverse() * vec ;

  finalTargetInMapCoordinateSys.position.x = vecFinal (0) ;
  finalTargetInMapCoordinateSys.position.y = vecFinal (1) ;
  finalTargetInMapCoordinateSys.position.z = vecFinal (2) ;

  return true;
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "move_to_table_node");
  ROS_INFO("move_to_table node started....");
  MoveToTableNode mtt;
  while (ros::ok())
  {
    ros::spinOnce ();
  }
}
