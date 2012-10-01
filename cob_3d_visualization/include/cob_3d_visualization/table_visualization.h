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
