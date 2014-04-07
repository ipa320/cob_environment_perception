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
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_fov_segmentation
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 02/2013
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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

#ifndef __FOV_SEGMENTATION_NODE_H__
#define __FOV_SEGMENTATION_NODE_H__

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ROS message includes
#include <cob_3d_mapping_msgs/ShapeArray.h>

#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_fov_segmentation/field_of_view.h"
#include "cob_3d_fov_segmentation/fov_segmentation.h"

namespace cob_3d_mapping
{
  /**
   * @brief ROS node for FOV segmentation of a shape array.
   * @details Node handles input Shape arrays and provides output Shape arrays.\n
   * Input topics: /tf, /shape_array\n
   * Output topics: /shape_array
   */
  class FOVSegmentationNode
  {
  public:
    /**
     * @brief Constructor
     */
    FOVSegmentationNode ();
    /**
     * brief Destructor
     */
    ~FOVSegmentationNode ()
    {
    }
    ;

    /**
     * @brief Callback for dynamic reconfigure
     *
     * everytime the dynamic reconfiguration changes this function will be called
     *
     * @param inst instance of AggregatePointMap which parameters should be changed
     * @param config data of configuration
     * @param level bit descriptor which notifies which parameter changed
     *
     * @return nothing
     */
    void
    dynReconfCallback (cob_3d_fov_segmentation::fov_segmentationConfig &config, uint32_t level);

    /**
     * \brief Callback for the shape subscriber.
     *
     * \param[in] The incoming shape array.
     */
    void
    shapeCallback (const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa);

    ros::NodeHandle n_; ///< ROS node handle.

  protected:
    ros::Subscriber shape_sub_; ///< Subscription to shape message to be processed.
    ros::Publisher shape_pub_; ///< Publish Map array as shape message.

    tf::TransformListener tf_listener_; ///< Retrieves transformations.

    /**
     * @brief Dynamic Reconfigure server
     */
    dynamic_reconfigure::Server<cob_3d_fov_segmentation::fov_segmentationConfig> config_server_;

    FieldOfView fov_; ///< Map containing geometrys (polygons,cylinders)
    FOVSegmentation fov_seg_; ///< The FOV segmentation object.

    std::string camera_frame_; ///< The camera frame.
    std::string target_frame_; ///< The target frame.
  };
}

#endif // FOV_SEGMENTATION_NODE_H__
