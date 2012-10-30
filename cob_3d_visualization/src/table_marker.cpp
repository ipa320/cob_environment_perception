/*
 * table_marker.cpp
 *
 *  Created on: Sep 25, 2012
 *      Author: goa-sn
 */

#include <cob_3d_visualization/table_marker.h>


/**
 * @brief Publish interactive markers for a table detected by table_extractiion node using interactive marker server
 */
void TableMarker::createInteractiveMarkerForTable ()
{
  float offset(0.1);
  //  ctr_ ++;
  cob_3d_mapping::Polygon p;
  cob_3d_mapping::fromROSMsg (table_, p);

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
  createMarkerforTable (tri_list, im_ctrl);


  table_int_marker_.controls.push_back (im_ctrl);
  table_im_server_->insert (table_int_marker_);


  table_menu_handler_.apply (*table_im_server_, table_int_marker_.name);
  table_im_server_ ->applyChanges() ;

}
/**
 * @brief Create marker for the table and add it to the interactive marker control
 *
 * @param triangle_list triangulated list of poly points
 * @param im_ctrl interactive marker control
 *
 */
void
TableMarker::createMarkerforTable (list<TPPLPoly>& triangle_list, visualization_msgs::InteractiveMarkerControl& im_ctrl)
{
  TPPLPoint pt;
  for (std::list<TPPLPoly>::iterator it = triangle_list.begin (); it != triangle_list.end (); it++)
  {
    table_marker_.id = id_ ; //ctr_;

    table_marker_.header = table_.header;
    table_marker_.header.stamp = ros::Time::now() ;

    table_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    table_marker_.ns = "table visualization";
    table_marker_.action = visualization_msgs::Marker::ADD;
    table_marker_.lifetime = ros::Duration ();

    //set color
    table_marker_.color.r = 1 ;//shape_.color.r;
    table_marker_.color.g = 0;//shape_.color.g;
    table_marker_.color.b = 0;//shape_.color.b;
    table_marker_.color.a = 1;//shape_.color.a;

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
/**
 * @brief Create menu entries for each shape
 *
 * @param point 3D point to be transformed
 *
 * @return return transformed 2D TPPLPoint
 */
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
/**
 * @brief callback function when there is a mouse click on interactive marker
 *
 * @param feedback feedback from rviz when there is a mouse click on interactive marker
 */
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

