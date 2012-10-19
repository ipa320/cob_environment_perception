/*
 * table_visualization.h
 *
 *  Created on: Sep 19, 2012
 *      Author: goa-sn
 */

#ifndef TABLE_VISUALIZATION_H_
#define TABLE_VISUALIZATION_H_

//##################
//#### includes ####
// standard includes
#include <stdio.h>
#include <sstream>

// ros includes
#include <ros/ros.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/GetObjectsOfClass.h>
#include <cob_3d_mapping_msgs/GetTables.h>
#include <cob_3d_visualization/table_marker.h>

//#include <cob_3d_mapping_msgs/GetTables.h>


class TableVisualization
{
  public:
    // Constructor
    TableVisualization ()

    {
      ctr_ = 0 ;
      table_array_sub_ = nh_.subscribe ("table_array", 1, &TableVisualization::tableVisualizationCallback, this);
      table_im_server_.reset (new interactive_markers::InteractiveMarkerServer ("geometry_map/map", "", false));

    }
    // Destructor
    ~TableVisualization ()
    {

    }

    void tableVisualizationCallback (const cob_3d_mapping_msgs::ShapeArrayPtr& ta);



  protected:

    ros::NodeHandle nh_;
    ros::Subscriber table_array_sub_ ;
    std::vector<boost::shared_ptr<TableMarker> > v_tm_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> table_im_server_;



    int ctr_;
};

#endif /* TABLE_VISUALIZATION_H_ */
