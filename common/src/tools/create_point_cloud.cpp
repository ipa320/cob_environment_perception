/*
 * create_point_cloud.cpp
 *
 *  Created on: 15.06.2011
 *      Author: goa
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

//Create jump
/*int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	//cloud.points.resize(40000);
	cloud.height=20;
	cloud.width=20;
	int pt_ctr=0;
	double x=-0.2, y=0.2;
	for(int i=0; i<cloud.height; i++, y+=0.01)
	{
		x=0.2;
		for(int j=0; j<cloud.width; j++, x+=0.01, pt_ctr++)
		{
			pcl::PointXYZRGB p;
			p.x = x;
			p.y = y;
			p.rgb = 0;
			if(i<cloud.width/2) p.z=1.0;
			else p.z=1.2;
			cloud.points.push_back(p);
		}
	}
	pcl::io::savePCDFileASCII ("/home/goa/pcl_daten/cloud_sim.pcd", cloud);
}*/

//Create corner
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
	//cloud.points.resize(40000);
	cloud.height=20;
	cloud.width=20;
	int pt_ctr=0;
	double x=-0.1, y=0.2, z=1;
	double n_x=0, n_y=0, n_z=0;
	for(int i=0; i<cloud.height; i++, y+=0.01)
	{
		x=-0.1;
		z=1;
		n_z = -1;
		n_x = 0;
		for(int j=0; j<cloud.width; j++, x+=0.01, pt_ctr++)
		{
			if(x>=0)
			{
				x = 0;
				z -= 0.01;
				n_z = 0;
				n_x = -1;
			}

			pcl::PointXYZRGBNormal p;
			p.x = x;
			p.y = y;
			p.z = z;
			p.rgb = 0;
			p.normal_x = n_x;
			p.normal_y = n_y;
			p.normal_z = n_z;
			p.curvature = 0;
			//if(i<cloud.width/2) p.z=1.0;
			//else p.z=1.2;
			cloud.points.push_back(p);
		}
	}
	pcl::io::savePCDFileASCII ("/home/goa/pcl_daten/corner/corner_sim.pcd", cloud);
}
