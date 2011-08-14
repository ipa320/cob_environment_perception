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
      int ausgabe=0;
      int counter=0;
      double value_sum=0.0;

      switch (ausgabe)
      case x: ausgabe=1;
      case y: ausgabe=2;
      case z: ausgabe=3;
      for (int i=0 ; i< cloud_normals.size();i++)
      {
        if (0.8*soll< cloud_normals.points[i].normals[ausgabe] < 1.2*soll || -0.8*soll> cloud_normals.points[i].normals[ausgabe] > -1.2*soll)
        {
          counter++;
          value_sum=soll_all + cloud_normals.points[i].normals[ausgabe];
        }
      }
      ROS_INFO_STREAM("Abweichung von " << ausgabe << "ist " <<  (value_sum/counter)/soll );
    }
    void surfaceNormalsFast(PointCloud &cloud)
    {

    	boost::timer ti;


      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (cloud.makeShared());

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal> cloud_normals ;

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (cloud_normals);
     double time = ti.elapsed();
     pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n;
     pcl::concatenateFields (cloud, cloud_normals, cloud_n);

     pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/surfaceNormalsFast.pcd", cloud_n);
     ROS_INFO_STREAM("surface fast Time " <<time);


    }

void   IntergralImage(PointCloud &cloud){
		boost::timer ti;
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

		pcl::PointCloud<pcl::Normal> cloud_normals;

		ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(cloud.makeShared());
		ne.compute(cloud_normals);
		 double time = ti.elapsed();
		 pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n;
		 pcl::concatenateFields (cloud, cloud_normals, cloud_n);

		 pcl::io::savePCDFileASCII ("/home/goa-hh/pcl_daten/intergralImage.pcd", cloud_n);
		 ROS_INFO_STREAM("Integrate Time " <<time);

		 for(int i=0; i < cloud_normals.size();i++)
		 {

			if(0.8f < cloud_normals.points[i].normal[1] < 1.4f || -1.4f < cloud_normals.points[i].normal[1] < -0.8f ){
				ROS_INFO_STREAM("Punkte von normalen " <<cloud_normals.points[i].normal[1]);}
		 }
}
void surfaceNormals(PointCloud &cloud)
{

	boost::timer ti;


  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud.makeShared());

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal> cloud_normals ;

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (cloud_normals);
 double time = ti.elapsed();
 pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_n;
 pcl::concatenateFields (cloud, cloud_normals, cloud_n);

 pcl::io::savePCDFileASCII ("/home/goa-/pcl_daten/surfaceNormals.pcd", cloud_n);
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

			std::string directory("/home/heiko/pcl_daten/test/");
			PointCloud cloud_in;
			pcl::io::loadPCDFile(directory+"eingang_bin.pcd", cloud_in);
			ROS_INFO_STREAM("weite" <<cloud_in.width <<"höhe" << cloud_in.height);

			NormTime nt;
			nt.surfaceNormals(cloud_in);
			nt.surfaceNormalsFast(cloud_in);
			nt.IntergralImage(cloud_in);
    	return 0;
    }
