//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <tf_conversions/tf_kdl.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv.h>
#include <highgui.h>
#include "pcl/io/pcd_io.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/normal_3d.h"
#include <pcl/features/boundary.h>
//#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PointIndices.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/range_image/range_image.h>
#include <cob_env_model/features/range_image_border_extractor.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>

// external includes
#include <boost/timer.hpp>

//#include <cob_vision_features/SURFDetector.h>
//#include <cob_vision_features/AbstractFeatureVector.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PointT;

class TestRange
{
public:
    // Constructor
	TestRange()
	{
		/// void
	}

    // Destructor
    ~TestRange()
    {
    	/// void
    }
/*  add to actual pointcloud a new plane
 * of starting point x right an y up
 */
    void addNewPlane(PointCloud &cloud_in ,PointCloud &cloud_out , pcl::PointXYZ starting_point_xyz , int high , int width ,double distance )
    {
      cloud_out.resize(high * width + cloud_in.size());
      for(int i=0; i<cloud_in.size();i++)
      {
        cloud_out.points[i]=cloud_in.points[i];
      }
      int counter=cloud_in.size();
      double k = 0.0;
      double l = 0.0;
      for (int i=0; i<high;i++)
      {
        for(int j=0; j<width;j++)
        {
          if (counter<cloud_out.size()){
         cloud_out.points[counter].y=starting_point_xyz.y+l;
         cloud_out.points[counter].x=starting_point_xyz.x+ k;
         cloud_out.points[counter].z=starting_point_xyz.z;
         counter++;


          }


         k=k+distance;
        }
        k=0;
        l=l+distance;
      }
    }
    void parallelPlan(double distance , PointCloud &cloud)
    {

    	  // Fill in the cloud data
    	  cloud.width  = 400;
    	  cloud.height = 1;
    	  cloud.points.resize (cloud.width * cloud.height);
    	  double l=0;
    	  double k=0;
    	  int counter=0;

    	  // Generate the data
    	  for (int i = 0; i <= 4; i++)
    	  {

    		  for (int j=0;j<=40 ;j++)
    		  {
    			  cloud.points[counter].x=l;
    			  cloud.points[counter].y=k;
    			  cloud.points[counter].z=1;
    			  l=l+0.001;
    			  counter++;

    		  }
    		  k=k+0.001;
    		  l=0;
    	  }



    	  for (int i = 0; i <= 4; i++)
    	    	  {
    	    		  for (int j=0;j<=40 ;j++)
    	    		  {
    	    			  if(counter<400){
    	    			  cloud.points[counter].x=l;
    	    			  cloud.points[counter].y=k;
    	    			  cloud.points[counter].z=1+distance;
    	    			  l=l+0.001;
    	    			  counter++;}
    	    			  else{}
    	    		  }
    	    		  k=k+0.001;
    	    		  l=0;

    	    	  }


}

	void extractEdgesRangeImage(PointCloud& cloud_in, pcl::PointCloud<pcl::PointWithRange>& cloud_out, cv::Mat& border_image ,int radius , float border)
	{

		//pcl::RangeImage range_image;
		Eigen::Affine3f sensorPose =
		  (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

		//cloud_out.createFromPointCloud2 (*cloud_in, pcl::deg2rad(0.089f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), sensorPose);
		pcl::RangeImage range_image;
		range_image.header = cloud_in.header;
		range_image.width = cloud_in.width;
		range_image.height = cloud_in.height;
		range_image.is_dense = false;
		for(int i = 0; i<cloud_in.size(); i++)
		{
			pcl::PointWithRange p;

			//p.range = sqrt(cloud_in->points[i].x*cloud_in->points[i].x+cloud_in->points[i].y*cloud_in->points[i].y+cloud_in->points[i].z*cloud_in->points[i].z);
			/*if(cloud_in->points[i].x!=cloud_in->points[i].x || cloud_in->points[i].y!=cloud_in->points[i].y || cloud_in->points[i].z!=cloud_in->points[i].z)
			{
				p.x = p.y = p.z = 0;//std::numeric_limits<float>::quiet_NaN();
				p.range=0;//-std::numeric_limits<float>::infinity();
			}
			else*/
			{
				p.x = cloud_in.points[i].x;
				p.y = cloud_in.points[i].y;
				p.z = cloud_in.points[i].z;
				p.range = sqrt(cloud_in.points[i].x*cloud_in.points[i].x+cloud_in.points[i].y*cloud_in.points[i].y+cloud_in.points[i].z*cloud_in.points[i].z);
			}
			range_image.points.push_back(p);
		}
		//std::string directory("/home/goa/pcl_daten/test/");
		//pcl::io::savePCDFileASCII (directory+"/range_image.pcd", range_image);



		  ipa_features::RangeImageBorderExtractor border_extractor(&range_image);
		  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
		  border_extractor.setPixelRradiusBorders(radius);
		  border_extractor.setMinimumBorderProbability(border);


		  border_extractor.compute(border_descriptions);


		  pcl::PointCloud<pcl::Boundary>::Ptr boundary_pts (new pcl::PointCloud<pcl::Boundary> ());
		  //boundary_pts.points.resize(cloud_in.size());
		  for (int y=0; y<(int)range_image.height; ++y)
		  {
		    for (int x=0; x<(int)range_image.width; ++x)
		    {
		      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
		      {
		        cloud_out.points.push_back(range_image.points[y*range_image.width + x]);
		        pcl::Boundary p;
		        p.boundary_point = 1;
		        boundary_pts->points.push_back(p);
		      }
		      else
		      {
			        pcl::Boundary p;
			        p.boundary_point = 0;
			        boundary_pts->points.push_back(p);
		      }
		    }
		  }

			border_image = cv::Mat(range_image.height, range_image.width, CV_8UC1);
			int pt_idx=0;
			for(int row=0; row<border_image.rows; row++)
			{
				for(int col=0; col<border_image.cols; col++, pt_idx++)
				{
					if( boundary_pts->points[pt_idx].boundary_point == 1)
						border_image.at<unsigned char>(row,col) = 255;
					else
						border_image.at<unsigned char>(row,col) = 0;
				}
			}
	    	ROS_INFO_STREAM("size" << cloud_in.size());

	    	ROS_INFO_STREAM("size" << cloud_out.size());

	    	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	    	viewer.addCoordinateSystem(0.5f);
	    	viewer.addPointCloud<pcl::PointXYZ>(cloud_in.makeShared(), "input cloud");
	        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> single_color(cloud_out.makeShared(), 0, 255, 0);
	    	viewer.addPointCloud<pcl::PointWithRange>(cloud_out.makeShared(),single_color, "output cloud");

	    	while(!viewer.wasStopped())
	    	{
	    	viewer.spinOnce(100);
	    	usleep(100000);
	    	}
	}
};
    int main(int argc, char** argv)
    {
    	PointCloud input_cloud;
        PointCloud output_cloud;
        PointCloud output2_cloud;


    	TestRange tr;
    	pcl::PointXYZ starting_point(0,0,2);
        pcl::PointXYZ starting_point2(1,0,1);

    	//tr.parallelPlan(0.2 , input_cloud);
    	tr.addNewPlane(input_cloud, output_cloud ,starting_point , 100, 100,0.01);
    	tr.addNewPlane(output_cloud,output2_cloud, starting_point2,100, 100 ,0.01);

    	cv::Mat border_image;
    	pcl::PointCloud<pcl::PointWithRange> cloud_out;

    	tr.extractEdgesRangeImage(output2_cloud , cloud_out ,border_image , 1 , 0.2);
    	tr.extractEdgesRangeImage(output2_cloud , cloud_out ,border_image , 3 , 0.5);







    	return 0;
    }
