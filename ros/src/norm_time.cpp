//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "pcl/io/pcd_io.h"
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <cob_env_model/features/range_image_border_extractor.h>
#include <pcl/range_image/range_image.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

//#include <cob_vision_features/SURFDetector.h>
//#include <cob_vision_features/AbstractFeatureVector.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointType;

class NormTime
{
public:
    // Constructor
	NormTime()
	{
		/// void
	}

    // Destructor
    ~NormTime()
    {
    	/// void
    }

    /*
     * ausgabe darf nur x , y oder z sein
     */
   void calcDeviation(pcl::PointCloud<pcl::Normal> cloud_normals , double soll , char ausgabe)

    {
      int cord=0;
      int counter=0;
      double value_sum=0.0;

      switch (ausgabe){
      case 'x': cord=0;
			   break;
      case 'y': cord=1;
				break;
      case 'z': cord=2;
				break;
      }
      for (int i=0 ; i< cloud_normals.size();i++)
      {
        if (0.8*soll< cloud_normals.points[i].normal[cord] && cloud_normals.points[i].normal[cord]< 1.2*soll || -0.8*soll> cloud_normals.points[i].normal[cord] && cloud_normals.points[i].normal[cord] > -1.2*soll)
        {
          counter++;
          if ( cloud_normals.points[i].normal[cord] < 0)
          value_sum=value_sum - cloud_normals.points[i].normal[cord];
          else
              value_sum=value_sum + cloud_normals.points[i].normal[cord];

        }
      }
      ROS_INFO_STREAM("Durchschnittswert von " << ausgabe << " ist " <<  (value_sum/counter) << " Anzahl an guten normalen " << counter);
      ROS_INFO_STREAM("Abweichung von " << ausgabe << " ist " <<  (value_sum/counter)/soll );
    }


   /* void extractRangeImage (PointCloud &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals)
    {
    	pcl::RangeImage range_image;
    			range_image.header = cloud.header;
    			range_image.width = cloud.width;
    			range_image.height = cloud.height;
    			range_image.is_dense = false;
    			for(int i = 0; i<cloud.size(); i++)
    			{
    				pcl::PointWithRange p;

    				{
    					p.x = cloud.points[i].x;
    					p.y = cloud.points[i].y;
    					p.z = cloud.points[i].z;
    					p.range = sqrt(cloud.points[i].x*cloud.points[i].x+cloud.points[i].y*cloud.points[i].y+cloud.points[i].z*cloud.points[i].z);
    				}
    				range_image.points.push_back(p);
    			}

    			  ipa_features::RangeImageBorderExtractor border_extractor(&range_image);
    			  border_extractor=border_extractor.getSurfaceStructure();
    			  ROS_INFO_STREAM(border_extractor.LocalSurface);
    		/*	for (int i =0 ; i<border_extractor.LocalSurface->normal.size();i++)
    			{
    				Eigen::Vector3f normal = border_extractor.LocalSurface.normal;
    				cloud_normals.points[i].normal[0]=**border_extractor.LocalSurface.normal[0];
    				cloud_normals.points[i].normal[1]=border_extractor.LocalSurface->normal[1];
    				cloud_normals.points[i].normal[2]=border_extractor.LocalSurface->normal[2];


    			}/*
    			for(int i=0; i < cloud_normals.size();i++)
    												{

    														ROS_INFO_STREAM("Punkte von normalen " <<cloud_normals.points[i].normal);
    												}
*/

    }
    void surfaceNormalsFast(PointCloud &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals)
    {



      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (cloud.makeShared());

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      // Output datasets
      //pcl::PointCloud<pcl::Normal> cloud_normals ;

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);
      boost::timer ti;

      // Compute the features
      ne.compute (cloud_normals);
     double time = ti.elapsed();
    /* pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n;
     pcl::concatenateFields (cloud, cloud_normals, cloud_n);

     pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/surfaceNormalsFast.pcd", cloud_n);*/
     ROS_INFO_STREAM("surface fast Time " <<time);


    }

void   IntergralImage(PointCloud &cloud, pcl::PointCloud<pcl::Normal> &cloud_normals){
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

		//pcl::PointCloud<pcl::Normal> cloud_normals;

		ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(cloud.makeShared());
		boost::timer ti;
		ne.compute(cloud_normals);
		double time = ti.elapsed();
	/*	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n;
		pcl::concatenateFields (cloud, cloud_normals, cloud_n);

		pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/intergralImage.pcd", cloud_n);*/
		ROS_INFO_STREAM("Integrate Time " <<time);

	/*	for(int i=0; i < cloud_normals.size();i++)
			{

				if(0.8 < cloud_normals.points[i].normal[1] && cloud_normals.points[i].normal[1] < 1.4 || -1.4 < cloud_normals.points[i].normal[1] && cloud_normals.points[i].normal[1] < -0.8 ){
					ROS_INFO_STREAM("Punkte von normalen " <<cloud_normals.points[i].normal[1]);}
			}*/
}
void surfaceNormals(PointCloud &cloud,pcl::PointCloud<pcl::Normal> &cloud_normals)
{



  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud.makeShared());

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  //pcl::PointCloud<pcl::Normal> cloud_normals ;

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);
	boost::timer ti;

  // Compute the features
  ne.compute (cloud_normals);
 double time = ti.elapsed();
/* pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n;
 pcl::concatenateFields (cloud, cloud_normals, cloud_n);

 pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/surfaceNormals.pcd", cloud_n);*/
 ROS_INFO_STREAM("surface Time " <<time);


}

};
    int main(int argc, char** argv)
    {
    	// Für rechte wand
    	// x = 1,103377919
    	// z= 2,366201583

    	//gegenüberliegende wand
    	//x=0,422618262
    	//z=0,906307787

			std::string directory("/home/goa-hh/pcl_daten/");
			PointCloud cloud_in;
			pcl::io::loadPCDFile(directory+"simple_planes.pcd", cloud_in);

			NormTime nt;
			pcl::PointCloud<pcl::Normal> cloud_normals_surface;
			pcl::PointCloud<pcl::Normal> cloud_normals_surface_fast;
			pcl::PointCloud<pcl::Normal> cloud_normals_integral;

			nt.surfaceNormals(cloud_in,cloud_normals_surface);
			nt.surfaceNormalsFast(cloud_in,cloud_normals_surface_fast);
			nt.IntergralImage(cloud_in,cloud_normals_integral);

			/*for(int i=0; i < cloud_normals_surface_fast.size();i++)
						{

								ROS_INFO_STREAM("Punkte von normalen " <<cloud_normals_surface_fast.points[i]);
						}*/
			ROS_INFO("Surface  rechte Wand");
			nt.calcDeviation(cloud_normals_surface , 1.103377919 ,'x');
			nt.calcDeviation(cloud_normals_surface , 0.0 ,'y');
			nt.calcDeviation(cloud_normals_surface , 2.366201583 ,'z');
			ROS_INFO("--------------------------------------");
			ROS_INFO("Surface  vordere Wand");
			nt.calcDeviation(cloud_normals_surface , 0.422618262 ,'x');
			nt.calcDeviation(cloud_normals_surface , 0.0 ,'y');
			nt.calcDeviation(cloud_normals_surface , 0.906307787 ,'z');
			ROS_INFO("--------------------------------------");
			ROS_INFO("Surface fast rechte Wand");
			nt.calcDeviation(cloud_normals_surface_fast , 1.103377919 ,'x');
			nt.calcDeviation(cloud_normals_surface_fast , 0 ,'y');
			nt.calcDeviation(cloud_normals_surface_fast , 2.366201583 ,'z');
			ROS_INFO("--------------------------------------");
			ROS_INFO("Surface fast vordere Wand");
			nt.calcDeviation(cloud_normals_surface_fast , 0.422618262 ,'x');
			nt.calcDeviation(cloud_normals_surface_fast , 0.0 ,'y');
			nt.calcDeviation(cloud_normals_surface_fast , 0.906307787 ,'z');
			ROS_INFO("--------------------------------------");
			ROS_INFO("Integral fast rechte Wand");
			nt.calcDeviation(cloud_normals_integral , 1.103377919 ,'x');
			nt.calcDeviation(cloud_normals_integral , 0.0 ,'y');
			nt.calcDeviation(cloud_normals_integral , 2.366201583 ,'z');
			ROS_INFO("--------------------------------------");
			ROS_INFO("Integral fast vordere Wand");
			nt.calcDeviation(cloud_normals_integral , 0.422618262 ,'x');
			nt.calcDeviation(cloud_normals_integral , 0.0 ,'y');
			nt.calcDeviation(cloud_normals_integral , 0.906307787 ,'z');

	/*		for(int i=0; i < cloud_normals_surface_fast.size();i++)
									{

											ROS_INFO_STREAM("Punkte von normalen " <<cloud_normals_surface_fast.points[i].normal[2]);
									}
*/
    	return 0;
    }
