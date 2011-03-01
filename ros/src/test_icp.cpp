
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
 * ROS package name: pcl_test
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by:
 *
 * Date of creation: 09/2010
 * ToDo:
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

//##################
//#### includes ####

// standard includes
//--


#include <pcl/ros/register_point_struct.h>
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

#include "pcl/registration/icp.h"

#include <boost/timer.hpp>




//####################
//#### node class ####
class PCLTest
{
	public:

		// Constructor
		PCLTest()
		{
			num_frames_ = 30;
			first_ = true;
		}


		// Destructor
		~PCLTest()
		{
			/// void
		}

		void RegisterICP();

		int num_frames_;
		bool first_;
		pcl::PointCloud<pcl::PointXYZ> cloud_target_;
		pcl::PointCloud<pcl::PointXYZ> map_no_icp_;

};





void PCLTest::RegisterICP()
{
	for(int i = 0; i<num_frames_; i++)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud_source; // the pc to register
		//pcl::PointCloud<pcl::PointXYZ> cloud_target; // the map

		std::stringstream ss, ss1, ss2;
		ss << "frame_" << i << ".pcd";
		ss1 << "frame_" << i+1 << ".pcd";

		pcl::PCDReader reader;
		if(first_)
		{
			reader.read (ss.str(), cloud_target_);
			reader.read (ss.str(), map_no_icp_);
			first_ = false;
		}
		reader.read (ss1.str(), cloud_source);

		pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
		icp.setInputCloud(cloud_source.makeShared());
		icp.setInputTarget(cloud_target_.makeShared());
		icp.setMaximumIterations(50);
		icp.setMaxCorrespondenceDistance(0.1);
		icp.setTransformationEpsilon (1e-6);
		pcl::PointCloud<pcl::PointXYZ> cloud_source_aligned;
		boost::timer t;
		icp.align(cloud_source_aligned);
		cloud_target_ += cloud_source_aligned;
		map_no_icp_ += cloud_source;
		ROS_INFO("\tTime: %f", t.elapsed());
		std::cout << "ICP converged: " << icp.hasConverged() << std::endl;
		std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
		ss2 << "aligned_frame_" << i+1 << ".pcd";
		pcl::io::savePCDFileASCII (ss2.str(), cloud_source_aligned);
	}
	pcl::io::savePCDFileASCII ("map.pcd", cloud_target_);
	pcl::io::savePCDFileASCII ("map_no_icp.pcd", map_no_icp_);
}






//#######################
//#### main programm ####




int main(int argc, char** argv)
{


	PCLTest pclTest;

	pclTest.RegisterICP();

    std::cout << "done" << std::endl;


    return 0;
}


