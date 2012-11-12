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
 *\author
 * Author: Shaghayegh Nazari, email:georg.arbeiter@ipa.fhg.de
 * \author
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 09/2012
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

#include <cob_3d_visualization/table_marker.h>

TableMarker::TableMarker (boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,cob_3d_mapping_msgs::Shape& table,int ctr,
    tabletop_object_detector::Table& tableMsg)
{
  id_ = ctr ;
  table_im_server_ = server ;
  table_ = table ;
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_linear_simple/goal", 1);

  createTableMenu();
  createInteractiveMarkerForTable();
  //Table msg
  table_msg_ = tableMsg ;
}

void TableMarker::createInteractiveMarkerForTable ()
{
  cob_3d_mapping::Polygon p;
  cob_3d_mapping::fromROSMsg (table_, p);



  /* create interactive marker for *this shape */
  stringstream ss;
  ss << "table_"<< id_ ; //ctr_ ;
  table_int_marker_.name = ss.str ();
  table_int_marker_.header = table_.header;
  table_int_marker_.header.stamp = ros::Time::now() ;

  ss.str ("");
  im_ctrl.always_visible = true;
  ss << "table_" << id_ << "_control";
  im_ctrl.name = ss.str ();
  im_ctrl.description = "table_markers";
  im_ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  /* create marker */
  createMarkerforTable (im_ctrl);

  table_int_marker_.controls.push_back (im_ctrl);
  table_im_server_->insert (table_int_marker_);


  table_menu_handler_.apply (*table_im_server_, table_int_marker_.name);
  table_im_server_ ->applyChanges() ;

}

void
TableMarker::createMarkerforTable (visualization_msgs::InteractiveMarkerControl& im_ctrl)
{
  float offset(0.1);

  //  /* transform shape points to 2d and store 2d point in triangle list */
  TPPLPartition pp;
  list<TPPLPoly> polys, tri_list;

  Eigen::Vector3f v, normal, origin;
  if (table_.params.size () == 4)
  {
    normal (0) = table_.params[0];
    normal (1) = table_.params[1];
    normal (2) = table_.params[2];
    origin (0) = table_.centroid.x + offset ;
    origin (1) = table_.centroid.y + offset ;
    origin (2) = table_.centroid.z + offset ;
    v = normal.unitOrthogonal ();

    pcl::getTransformationFromTwoUnitVectorsAndOrigin (v, normal, origin, transformation_);
    transformation_inv_ = transformation_.inverse ();
  }

  for (size_t i = 0; i < table_.points.size (); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> pc;
    TPPLPoly poly;
    pcl::fromROSMsg (table_.points[i], pc);
    poly.Init (pc.points.size ());
    poly.SetHole (table_.holes[i]);

    for (size_t j = 0; j < pc.points.size (); j++)
    {
      poly[j] = msgToPoint2DforTable (pc[j]);
    }
    if (table_.holes[i])
      poly.SetOrientation (TPPL_CW);
    else
      poly.SetOrientation (TPPL_CCW);

    polys.push_back (poly);
  }
  pp.Triangulate_EC (&polys, &tri_list);
  TPPLPoint pt;
  for (std::list<TPPLPoly>::iterator it = tri_list.begin (); it != tri_list.end (); it++)
  {
    table_marker_.id = id_ ; //ctr_;

    table_marker_.header = table_.header;
    table_marker_.header.stamp = ros::Time::now() ;

    table_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    table_marker_.ns = "table visualization";
    table_marker_.action = visualization_msgs::Marker::ADD;
    table_marker_.lifetime = ros::Duration ();

    //set color
    table_marker_.color.r = 1 ;
    table_marker_.color.g = 0;
    table_marker_.color.b = 0;
    table_marker_.color.a = 1;

    //set scale
    table_marker_.scale.x = 1;
    table_marker_.scale.y = 1;
    table_marker_.scale.z = 1;

    //set pose
    Eigen::Quaternionf quat (transformation_inv_.rotation ());
    Eigen::Vector3f trans (transformation_inv_.translation ());

    table_marker_.pose.position.x = trans (0);
    table_marker_.pose.position.y = trans (1);
    table_marker_.pose.position.z = trans (2);

    table_marker_.pose.orientation.x = quat.x ();
    table_marker_.pose.orientation.y = quat.y ();
    table_marker_.pose.orientation.z = quat.z ();
    table_marker_.pose.orientation.w = quat.w ();

    //draw each triangle
    table_marker_.points.resize (it->GetNumPoints ());
    for (long i = 0; i < it->GetNumPoints (); i++)
    {
      pt = it->GetPoint (i);
      table_marker_.points[i].x = pt.x;
      table_marker_.points[i].y = pt.y;
      table_marker_.points[i].z = 0;
    }
    im_ctrl.markers.push_back (table_marker_);
  }
}

TPPLPoint
TableMarker::msgToPoint2DforTable (const pcl::PointXYZ &point)
{
  //ROS_INFO(" transform 3D point to 2D ");
  TPPLPoint pt;
  Eigen::Vector3f p = transformation_ * point.getVector3fMap ();
  pt.x = p (0);
  pt.y = p (1);
  //std::cout << "\n transformed point : \n" << p << std::endl;
  return pt;
}

void TableMarker::tableFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {

  ROS_INFO("%s position : x= %f, y= %f, z= %f", table_int_marker_.name.c_str(), table_.centroid.x,table_.centroid.y,table_.centroid.z);

}
void TableMarker::createTableMenu() {

  interactive_markers::MenuHandler::EntryHandle eh_1;
  eh_1 = table_menu_handler_.insert ("Move to this table",boost::bind (&TableMarker::MoveToTheTable, this, _1));

  table_menu_handler_.setVisible (eh_1, true);
  table_menu_handler_.setCheckState (eh_1, interactive_markers::MenuHandler::NO_CHECKBOX);
}

void TableMarker::MoveToTheTable(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  std::cout << "in MovingToTable..." << "\n" ;

  ROS_INFO("Moving to table %d...",id_) ;

  cob_3d_mapping_msgs::MoveToTable::Request  reqMoveToTable;
  cob_3d_mapping_msgs::MoveToTable::Response resMoveToTable;

  reqMoveToTable.targetTable = table_msg_ ;
  reqMoveToTable.tableCentroid.position.x = table_.centroid.x ;
  reqMoveToTable.tableCentroid.position.y = table_.centroid.y ;
  reqMoveToTable.tableCentroid.position.z = table_.centroid.z ;

  if (ros::service::call("/move_to_table",reqMoveToTable,resMoveToTable)){
    // Calling move_to_table Service...
  }

}

