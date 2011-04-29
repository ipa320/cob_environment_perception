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
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 04/2011
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


class ConcatFields
{
public:
	typedef sensor_msgs::PointCloud2 PointCloud2;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

	ConcatFields()
	{
		sub_input_filter_.subscribe(nh_, "input", 3);
		sub_cloud2_filter_.subscribe(nh_, "input2", 3);

		sync_input_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> > >(3);
		sync_input_->connectInput(sub_input_filter_, sub_cloud2_filter_);
		sync_input_->registerCallback (boost::bind (&ConcatFields::input_callback, this, _1, _2));

		pub_output_ = nh_.advertise<sensor_msgs::PointCloud2> ("output", 3);
	}

	void input_callback(const PointCloud2::ConstPtr &cloud, const PointCloud2::ConstPtr &cloud2)
	{
		ROS_INFO("[ConcatNormals] SyncCallback");
		ROS_INFO("width: %d, %d", cloud->width,cloud2->width);
		ROS_INFO("stamp: %f, %f", cloud->header.stamp.toSec(),cloud2->header.stamp.toSec());
		PointCloud2 cloud_out;
		if(cloud->height*cloud->width != cloud2->height*cloud2->width)
		{
			ROS_ERROR("[input_callback] Error: Point cloud and cloud2 cloud do not have the same size.");
			return;
		}
		cloud_out = *cloud;
		// Resize the output dataset
		int data_size = cloud_out.data.size ();
		int nr_fields = cloud_out.fields.size ();
		int nr_points = cloud_out.width * cloud_out.height;

		// Point step must increase with the length of each new field
		cloud_out.point_step += cloud2->point_step;
		// Resize data to hold all clouds
		data_size += cloud2->data.size ();

		// Concatenate fields
		cloud_out.fields.resize (nr_fields + cloud2->fields.size ());
		int delta_offset = cloud->point_step;//cloud_out.fields[nr_fields - 1].offset + pcl::getFieldSize (cloud_out.fields[nr_fields - 1].datatype);
		for (size_t d = 0; d < cloud2->fields.size (); ++d)
		{
		  cloud_out.fields[nr_fields + d] = cloud2->fields[d];
		  cloud_out.fields[nr_fields + d].offset += delta_offset;
		}
		/*for (size_t d = 0; d < cloud->fields.size (); d++)
		{
			ROS_INFO("Field name: %s, offset: %d", cloud->fields[d].name.c_str(),cloud->fields[d].offset);
		}
		ROS_INFO("Point step: %d\n", cloud->point_step);
		for (size_t d = 0; d < cloud2->fields.size (); d++)
		{
			ROS_INFO("Field name: %s, offset: %d", cloud2->fields[d].name.c_str(),cloud2->fields[d].offset);
		}
		ROS_INFO("Point step: %d\n", cloud2->point_step);
		for (size_t d = 0; d < cloud_out.fields.size (); d++)
		{
			ROS_INFO("Field name: %s, offset: %d", cloud_out.fields[d].name.c_str(),cloud_out.fields[d].offset);
		}
		ROS_INFO("Point step: %d", cloud_out.point_step);*/
		//nr_fields += cloud2->fields.size ();

	  // Recalculate row_step
	  cloud_out.row_step = cloud_out.point_step * cloud_out.width;
	  cloud_out.data.resize (data_size);

	  // Iterate over each point and perform the appropriate memcpys
	  int point_offset = delta_offset;//0;
	  int point_offset2 = 0;
	  for (int cp = 0; cp < nr_points; ++cp)
	  {
		  // Copy each individual point
		  memcpy (&cloud_out.data[point_offset], &cloud2->data[cp * cloud2->point_step], cloud2->point_step);
		  point_offset += cloud_out.point_step;
		  memcpy (&cloud_out.data[point_offset2], &cloud->data[cp * cloud->point_step], cloud->point_step);
		  point_offset2 += cloud_out.point_step;
	  }
	  pub_output_.publish (boost::make_shared<const PointCloud2> (cloud_out));

	}

	ros::NodeHandle nh_;
	message_filters::Subscriber<PointCloud2> sub_input_filter_;
	message_filters::Subscriber<PointCloud2> sub_cloud2_filter_;
	ros::Publisher pub_output_;
	boost::shared_ptr <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> > > sync_input_;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "concat_cloud2_node");
	ConcatFields cf;

	ros::spin();

	return 0;
}
