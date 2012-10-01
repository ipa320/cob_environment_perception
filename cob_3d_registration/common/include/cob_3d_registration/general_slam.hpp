/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: dynamic_tutorials
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: goa-jh
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Oct 26, 2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

template <class Point>
class GeneralSLAM<Point>
{
public:
  GeneralSLAM():odometry_(Eigen::Matrix4f::Identity())
  {}

  virtual ~GeneralSLAM() {}

  virtual void  setInputCloud (const PointCloudConstPtr &cloud) {
    input_ = cloud;
  }

  virtual void  setOdometry (const Eigen::Matrix4f &odometry) {
    odometry_ = odometry;
  }

  virtual pcl::PointCloud<Point>::ConstPtr getMap() {
    return map_;
  }

  virtual bool compute() {
    if(!compute_registration())
      return false;
    if(!compute_transformation())
      return false;
    if(!compute_map())
      return false;
  }

protected:

  virtual bool compute_registration()=0;
  virtual bool compute_transformation()=0;
  virtual bool compute_map()=0;

  PointCloudConstPtr input_;
  pcl::PointCloud<Point> map_;
  Eigen::Matrix4f odometry_;

};
