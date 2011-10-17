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

//Create vertical corner
/*int main(int argc, char** argv)
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
				x -= 0.009;
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
}*/

//create edge cloud
/*int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::Boundary>::Ptr cloud = pcl::PointCloud<pcl::Boundary>::Ptr (new pcl::PointCloud<pcl::Boundary>);
	//cloud.points.resize(40000);
	cloud->height=20;
	cloud->width=20;
	cloud->resize(20*20);
	for(unsigned int i=0; i<cloud->height; i++)
	{
		for(unsigned int j=0; j<cloud->width; j++)
		{
			if(i==4)
				cloud->points[i*cloud->width+j].boundary_point = 1;
			else if(i==12)
				cloud->points[i*cloud->width+j].boundary_point = 1;
			else
			{
				cloud->points[i*cloud->width+j].boundary_point = 0;
			}
		}
	}
	pcl::io::savePCDFile ("/home/goa/pcl_daten/corner/contour_sim.pcd", *cloud, true);
}*/

//create horizontal corner
/*int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
	//cloud.points.resize(40000);
	cloud.height=20;
	cloud.width=20;
	int pt_ctr=0;
	double x=-0.1, y=-0.1, z=1;
	double n_x=0, n_y=0, n_z=0;
	for(int i=0; i<cloud.height; i++, y+=0.01)
	{
		if(y>=0)
		{
			y -= 0.009;
			z -= 0.01;
			n_z = 0;
			n_x = -1;
		}
		x=-0.1;
		//z=1;
		n_z = -1;
		n_x = 0;
		for(int j=0; j<cloud.width; j++, x+=0.01, pt_ctr++)
		{

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
	pcl::io::savePCDFileASCII ("/home/goa/pcl_daten/corner/corner_sim_h.pcd", cloud);
}*/

//create wall and floor
int main(int argc, char** argv)
{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        //cloud.points.resize(40000);
        cloud->height=30;
        cloud->width=30;
        cloud->resize(30*30);
        double x=0, y=0, z=0;
        for(unsigned int i=0; i<cloud->height; i++, y+=0.02)
        {
          x=0;
                for(unsigned int j=0; j<cloud->width; j++, x+=0.02)
                {
                  cloud->points[i*cloud->width+j].x = x;
                        if(j<=10)
                        {
                                cloud->points[i*cloud->width+j].y = 0;
                                cloud->points[i*cloud->width+j].z = z;
                        }
                        else
                        {
                          cloud->points[i*cloud->width+j].y = y;
                          cloud->points[i*cloud->width+j].z = 0;
                        }
                }
                z+=0.05;
        }
        pcl::io::savePCDFile ("/home/goa/pcl_daten/corner/wall_floor.pcd", *cloud, false);
}
