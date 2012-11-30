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
#if 1
/*
 * keyframe_detector.cpp
 *
 *  Created on: Sep 22, 2011
 *      Author: goa-jh
 */



//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <fstream>


// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cob_env_model/point_types.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <cob_env_model/field_of_view_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


// ROS message includes
#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model_msgs/GetFieldOfView.h>
#include "cob_env_model_msgs/TriggerMappingAction.h"
#include <cob_env_model_msgs/SetReferenceMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "cob_env_model/map/point_map.h"

#include "reconfigureable_node.h"
#include <cob_env_model/keyframe_detectorConfig.h>

// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <cob_env_model_msgs/GetFieldOfView.h>
#include "cob_env_model_msgs/TriggerMappingAction.h"
#include <cob_env_model_msgs/SetReferenceMap.h>
#include <cob_srvs/Trigger.h>

// external includes
#include <boost/timer.hpp>
#include <boost/numeric/ublas/matrix.hpp>

//#include "cob_env_model/filters/amplitude_filter.h"
#include "cob_env_model/filters/impl/jump_edge_filter.hpp"
#include "cob_env_model/filters/impl/amplitude_filter.hpp"
#include "cob_env_model/filters/impl/speckle_filter.hpp"
#include <sstream>

using namespace tf;

//####################
//#### node class ####
class KeyframeDetector
{
  typedef pcl::PointXYZRGB Point;

public:
  // Constructor
  KeyframeDetector()
  {
    point_cloud_pub1_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_cut",1);
    point_cloud_pub2_ = n_.advertise<pcl::PointCloud<Point> >("point_cloud2_filtered",1);
    point_cloud_sub_ = n_.subscribe("/tof/point_cloud2", 1, &KeyframeDetector::pointCloudSubCallback, this);
  }


  // Destructor
  ~KeyframeDetector()
  {
    /// void
  }
  /*void
  pointCloudSubCallback(const pcl::PointCloud<Point>::Ptr& pc_in)
  {
    ROS_INFO("%d %d", pc_in->width, pc_in->height);

    for(int x=0; x<pc_in->width; x++) {
      for(int y=0; y<pc_in->height; y++) {

        Point &p = pc_in->points[((x)+(y)*pc_in->width)];

        //ROS_INFO("%f %f %f", p.x, p.y, p.z);

        if(p.z>2.3||p.x>0.35)
          p.z = INFINITY;

      }
    }

    point_cloud_pub_.publish(*pc_in);*/

#define TEST_FILTER 2
  void
  pointCloudSubCallback (sensor_msgs::PointCloud2::ConstPtr pc)
  {
    static int ctr=0;
    ctr++;
#if TEST_FILTER==1
    std::string name="speckle";
    {int p1=50;//for(int p1=10; p1<=70; p1+=20) {
    {double p2=0.1;//for(double p2=0.01; p2<0.5; p2+=0.05) {
    cob_env_model::SpeckleFilter<sensor_msgs::PointCloud2> filter;
    filter.setFilterParam(p1,p2);
#elif TEST_FILTER==2

    std::string name="jump_edge";
    for(int p1=0; p1<=0; p1+=20) {
      {double p2=167;//for(double p2=50; p2<=189; p2+=9) {
    cob_env_model::JumpEdgeFilter<sensor_msgs::PointCloud2> filter;
    filter.setUpperAngle(p2);
#elif TEST_FILTER==3

    std::string name="amplitude";
    {double p1=0.001;//for(double p1=0.001; p1<=0.095; p1+=0.01) {
      //for(double p2=0.02; p2<0.2; p2+=0.02) {
      for(double p2=1.; p2<1.2; p2+=1.02) {
    cob_env_model::AmplitudeFilter<sensor_msgs::PointCloud2> filter;
    filter.setFilterLimits (p1, 1);
#elif TEST_FILTER==4

    std::string name="sor";
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;

    int p1=70;{//for(int p1=10; p1<=70; p1+=20) {
      {double p2=0.02;//for(double p2=0.01; p2<0.2; p2+=0.01) {
    filter.setMeanK(p1);
    filter.setStddevMulThresh(p2);

#endif

#if TEST_FILTER==4
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ> pc2;
    pcl::fromROSMsg(*pc,pc2);
    filter.setInputCloud (pc2.makeShared());
#else
    sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
    filter.setInputCloud (pc);
#endif
    //std::cout<<" min_limit :"<<filter.getFilterMinLimit()<<std::endl;
    //std::cout<<" max_limit :"<<filter.getFilterMaxLimit()<<std::endl;

    boost::timer t;
#if TEST_FILTER==4
    filter.filter (*cloud_filtered);
#else
    filter.filter (*cloud_filtered);
#endif
    ROS_INFO("[point_map] ICP took %f s", t.elapsed());

    ROS_INFO("%d %d", cloud_filtered->width, cloud_filtered->height);

    //*cloud_filtered = *pc;

    point_cloud_pub2_.publish (cloud_filtered);

    char fn[128];
    std::ostringstream of;
    {
      of<<name<<"/"<<p1<<"/"<<p2;
      system((std::string("mkdir -p ")+of.str()).c_str());
      of<<"/output"<<ctr<<".dat";
      FILE *fp = fopen(of.str().c_str(), "w");

      fputs("[",fp);
      for(int x=0; x<pc->width; x++) {
        if(x==0)
          fputs("[",fp);
        else
          fputs(",[",fp);
        for(int y=0; y<pc->height; y++) {

          float a,b,c=-100000.f;
          a=*(float*)&pc->data[(x+y*pc->width) * pc->point_step + 0];
          b=*(float*)&pc->data[(x+y*pc->width) * pc->point_step + 4];

#if TEST_FILTER==4
          for(int i=0; i<cloud_filtered->points.size(); i++) {
            if( *(float*)&pc->data[(x+y*pc->width) * pc->point_step + 0]==cloud_filtered->points[i].x &&
                *(float*)&pc->data[(x+y*pc->width) * pc->point_step + 4]==cloud_filtered->points[i].y) {
              c = cloud_filtered->points[i].z;
            }
          }
#else
          for(int i=0; i<cloud_filtered->data.size()/pc->point_step; i++) {
            if( *(float*)&pc->data[(x+y*pc->width) * pc->point_step + 0]==*(float*)&cloud_filtered->data[i * pc->point_step + 0] &&
                *(float*)&pc->data[(x+y*pc->width) * pc->point_step + 4]==*(float*)&cloud_filtered->data[i * pc->point_step + 4]) {
              c = *(float*)&cloud_filtered->data[i * pc->point_step + 8];
            }
          }
#endif

          if(y==0)
            sprintf(fn,"%f",c);
          else
            sprintf(fn,",%f",c);
          fputs(fn,fp);
        }

        fputs("]",fp);
      }
      fputs("]",fp);
      fclose(fp);
    }
      }
    }

    {
      char fn[128];
      sprintf(fn,"original%d.dat",ctr);
      FILE *fp = fopen(fn, "w");

      fputs("[",fp);
      for(int x=0; x<pc->width; x++) {
        if(x==0)
          fputs("[",fp);
        else
          fputs(",[",fp);
        for(int y=0; y<pc->height; y++) {

          float a,b,c=-100000.f;
          a=*(float*)&pc->data[(x+y*pc->width) * pc->point_step + 0];
          b=*(float*)&pc->data[(x+y*pc->width) * pc->point_step + 4];
          c=*(float*)&pc->data[(x+y*pc->width) * pc->point_step + 8];

          if(y==0)
            sprintf(fn,"%f",c);
          else
            sprintf(fn,",%f",c);
          fputs(fn,fp);
        }

        fputs("]",fp);
      }
      fputs("]",fp);
      fclose(fp);
    }
  }


  ros::NodeHandle n_;
  ros::Time stamp_;


protected:
  ros::Subscriber point_cloud_sub_;             //subscriber for input pc
  ros::Publisher point_cloud_pub1_, point_cloud_pub2_;              //publisher for map

  StampedTransform transform_old_;

  std::string frame_id_;


  double y_limit_;
  double distance_limit_;
  double r_limit_;
  double p_limit_;

  boost::mutex m_mutex_pointCloudSubCallback;

};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "keyframe_detector");

  KeyframeDetector kd;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    //fov.transformNormals();
    loop_rate.sleep();
  }
}


#elif 2

#include "cob_env_model/table_object_cluster.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <cob_env_model_msgs/GetFieldOfView.h>
#include <Eigen/Core>
#include "cob_env_model/map/point_map.h"


#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/Vertices.h>
#include "cob_env_model/features/plane_extraction.h"
#include "cob_env_model_msgs/PlaneExtractionAction.h"
#include <vector>


double testPointMap(pcl::PointCloud<PointMap::Point> pc, pcl::PointCloud<PointMap::Point> _pc_in, const pcl::PointCloud<PointMap::Point> * const ref_map, int max_it, double max_dist, double trf, bool reset, const std::string &fn_out, double &time) {
  pcl::PointCloud<PointMap::Point>::Ptr pc_in = _pc_in.makeShared();

  //setVerbosityLevel(pcl::console::L_DEBUG);

  static PointMap point_map_(0);

  if(reset)
    point_map_ = PointMap(0);

  if(!point_map_.getUseReferenceMap()) {
    *point_map_.getRefMap() = *ref_map;
    //point_map_.setReferenceMap((pcl::PointCloud<PointMap::Point>&)*ref_map);
    point_map_.setUseReferenceMap(true);

    point_map_.setICP_maxIterations(max_it);
    point_map_.setICP_maxCorrDist(max_dist);
    point_map_.setICP_maxFirstCorrDist(0.3);
    point_map_.setICP_trfEpsilon(trf);
    point_map_.setReuse(true);
  }

  double voxel=0.01;
  /*{
    pcl::VoxelGrid<PointMap::Point> vox_filter;
    vox_filter.setInputCloud(pc_in);
    vox_filter.setLeafSize(0.05,0.05,0.05);
    vox_filter.filter(_pc_in);
  }*/
  {
    pcl::VoxelGrid<PointMap::Point> vox_filter;
    vox_filter.setInputCloud(pc.makeShared());
    vox_filter.setLeafSize(voxel,voxel,voxel);
    vox_filter.filter(pc);
  }

  StampedTransform transform;
  if(point_map_.compute(pc_in, pc.makeShared())) {

    if(fn_out.size()>0) {
      pcl::VoxelGrid<PointMap::Point> vox_filter;
      vox_filter.setInputCloud(point_map_.getMap()->makeShared());
      vox_filter.setLeafSize(voxel,voxel,voxel);
      vox_filter.filter(*point_map_.getMap());

      pcl::io::savePCDFileASCII (fn_out.c_str(), *point_map_.getMap());
    }

    //ROS_INFO("Success");
  }

  time=point_map_.getComputionTime();

  return point_map_.getFitness();
}

int main (int argc, char** argv)
{
  //preload files
  std::vector<pcl::PointCloud<PointMap::Point>::Ptr> files,files_in;
  pcl::PointCloud<PointMap::Point> ref_map;

  //pcl::io::loadPCDFile("/home/goa-jh/bagfiles/kitchen_real_empty/icp/pc_in_trans_0.pcd", ref_map);
  pcl::io::loadPCDFile("/home/goa-jh/bagfiles/kitchen_sim_empty_n0/icp/map_28.pcd", ref_map);
  for(int loopCount=1; loopCount<9; loopCount++) {
    std::stringstream ss1;
    ss1 << "/home/goa-jh/bagfiles/kitchen_real_empty/icp/pc_trans_" << loopCount << ".pcd";
    std::stringstream ss2;
    ss2 << "/home/goa-jh/bagfiles/kitchen_real_empty/icp/pc_in_trans_" << loopCount << ".pcd";

    pcl::PointCloud<PointMap::Point> _pc_in, _pc;
    pcl::io::loadPCDFile(ss2.str().c_str(), _pc_in);
    files_in.push_back(_pc_in.makeShared());

    pcl::io::loadPCDFile(ss1.str().c_str(), _pc);
    files.push_back(_pc.makeShared());
  }
  /*
  {
    pcl::VoxelGrid<PointMap::Point> vox_filter;
    vox_filter.setInputCloud(ref_map.makeShared());
    vox_filter.setLeafSize(0.5,0.5,0.5);
    vox_filter.filter(ref_map);
  }*/


  int max_it=50;
  double max_dist=0.25, trf=0.0005;//1e-6;


  FILE *f=fopen("/home/goa-jh/log_apm_real_reuse.txt","w");
  //for(max_it=50; max_it<1000; max_it+=10)
  {
    //for(max_dist=0.05; max_dist<1.6; max_dist+=0.1) {
    //for(max_dist=0.1; max_dist<1.3; max_dist+=0.2)
    {
      //for(trf=1e-8; trf<1e-4; trf*=10)
      {

        double res=0;
        double time=0.;
        for(int loopCount=0; loopCount<files.size(); loopCount++) {
          double t=0.;

          std::stringstream ss3;
          ss3 << "/home/goa-jh/bagfiles/myown/mapo_"<<max_it<<"_"<<max_dist<<"_" << loopCount << ".pcd";

          res+=testPointMap(*files[loopCount],*files_in[loopCount], &ref_map, max_it, max_dist, trf, loopCount==0,ss3.str(),t);
          time+=t;

        }

        std::stringstream ss;
        ss<<max_it<<"\t"<<max_dist<<"\t"<<trf<<"\t"<<time<<"\t"<<res<<"\r\n";
        fputs(ss.str().c_str(), f);

      }

      fflush(f);
    }

    fclose(f);

    return 0;

  }
}
#else

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/ia_ransac.h"

class FeatureCloud
{
public:
  // A bit of shorthand
  typedef pcl::PointCloud<PointMap::Point> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
  typedef pcl::KdTreeFLANN<PointMap::Point> SearchMethod;

  FeatureCloud () :
    search_method_xyz_ (new SearchMethod),
    normal_radius_ (0.02),
    feature_radius_ (0.02)
  {}

  ~FeatureCloud () {}

  // Process the given cloud
  void
  setInputCloud (PointCloud::Ptr xyz)
  {
    xyz_ = xyz;
    processInput ();
  }

  // Load and process the cloud in the given PCD file
  void
  loadInputCloud (const std::string &pcd_file)
  {
    xyz_ = PointCloud::Ptr (new PointCloud);
    pcl::io::loadPCDFile (pcd_file, *xyz_);

    // Preprocess the cloud by...
    // ...removing distant points
    pcl::PassThrough<PointMap::Point> pass;
    pass.setInputCloud (xyz_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-10, 10);
    pass.filter (*xyz_);

    // ... and downsampling the point cloud
    const float voxel_grid_size = 0.01;
    pcl::VoxelGrid<PointMap::Point> vox_grid;
    vox_grid.setInputCloud (xyz_);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter (*xyz_);

    processInput ();
  }

  // Get a pointer to the cloud 3D points
  PointCloud::Ptr
  getPointCloud () const
  {
    return (xyz_);
  }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr
  getSurfaceNormals () const
  {
    return (normals_);
  }

  // Get a pointer to the cloud of feature descriptors
  LocalFeatures::Ptr
  getLocalFeatures () const
  {
    return (features_);
  }

protected:
  // Compute the surface normals and local features
  void
  processInput ()
  {
    computeSurfaceNormals ();
    computeLocalFeatures ();
  }

  // Compute the surface normals
  void
  computeSurfaceNormals ()
  {
    normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

    pcl::NormalEstimation<PointMap::Point, pcl::Normal> norm_est;
    norm_est.setInputCloud (xyz_);
    norm_est.setSearchMethod (search_method_xyz_);
    norm_est.setRadiusSearch (normal_radius_);
    norm_est.compute (*normals_);
  }

  // Compute the local feature descriptors
  void
  computeLocalFeatures ()
  {
    features_ = LocalFeatures::Ptr (new LocalFeatures);

    pcl::FPFHEstimation<PointMap::Point, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (xyz_);
    fpfh_est.setInputNormals (normals_);
    fpfh_est.setSearchMethod (search_method_xyz_);
    fpfh_est.setRadiusSearch (feature_radius_);
    fpfh_est.compute (*features_);
  }

private:
  // Point cloud data
  PointCloud::Ptr xyz_;
  SurfaceNormals::Ptr normals_;
  LocalFeatures::Ptr features_;
  SearchMethod::Ptr search_method_xyz_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
};

class TemplateAlignment
{
public:

  // A struct for storing alignment results
  struct Result
  {
    float fitness_score;
    Eigen::Matrix4f final_transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment () :
    min_sample_distance_ (0.05),
    max_correspondence_distance_ (0.01*0.01),
    nr_iterations_ (500)
  {
    // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
    sac_ia_.setMinSampleDistance (min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
    sac_ia_.setMaximumIterations (nr_iterations_);
  }

  ~TemplateAlignment () {}

  // Set the given cloud as the target to which the templates will be aligned
  void
  setTargetCloud (FeatureCloud &target_cloud)
  {
    target_ = target_cloud;
    sac_ia_.setInputTarget (target_cloud.getPointCloud ());
    sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
  }

  // Add the given cloud to the list of template clouds
  void
  addTemplateCloud (FeatureCloud &template_cloud)
  {
    templates_.push_back (template_cloud);
  }

  // Align the given template cloud to the target specified by setTargetCloud ()
  void
  align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
  {
    sac_ia_.setInputCloud (template_cloud.getPointCloud ());
    sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

    pcl::PointCloud<PointMap::Point> registration_output;
    sac_ia_.align (registration_output);

    result.fitness_score = sac_ia_.getFitnessScore (max_correspondence_distance_);
    result.final_transformation = sac_ia_.getFinalTransformation ();
  }

  // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
  void
  alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
  {
    results.resize (templates_.size ());
    for (size_t i = 0; i < templates_.size (); ++i)
    {
      align (templates_[i], results[i]);
    }
  }

  // Align all of template clouds to the target cloud to find the one with best alignment score
  int
  findBestAlignment (TemplateAlignment::Result &result)
  {
    // Align all of the templates to the target cloud
    std::vector<Result, Eigen::aligned_allocator<Result> > results;
    alignAll (results);

    // Find the template with the best (lowest) fitness score
    float lowest_score = std::numeric_limits<float>::infinity ();
    int best_template = 0;
    for (size_t i = 0; i < results.size (); ++i)
    {
      const Result &r = results[i];
      if (r.fitness_score < lowest_score)
      {
        lowest_score = r.fitness_score;
        best_template = i;
      }
    }

    // Output the best alignment
    result = results[best_template];
    return (best_template);
  }

private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  pcl::SampleConsensusInitialAlignment<PointMap::Point, PointMap::Point, pcl::FPFHSignature33> sac_ia_;
  float min_sample_distance_;
  float max_correspondence_distance_;
  float nr_iterations_;
};

// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{
  if (argc < 3)
  {
    printf ("No target PCD file given!\n");
    return (-1);
  }

  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
  std::ifstream input_stream (argv[1]);
  object_templates.resize (0);
  std::string pcd_filename;
  while (input_stream.good ())
  {
    std::getline (input_stream, pcd_filename);
    if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
      continue;

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud (pcd_filename);
    object_templates.push_back (template_cloud);
  }
  input_stream.close ();

  // Load the target cloud PCD file
  pcl::PointCloud<PointMap::Point>::Ptr cloud (new pcl::PointCloud<PointMap::Point>);
  pcl::io::loadPCDFile (argv[2], *cloud);

  // Preprocess the cloud by...
  // ...removing distant points
  pcl::PassThrough<PointMap::Point> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-10, 10);
  pass.filter (*cloud);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.01;
  pcl::VoxelGrid<PointMap::Point> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  vox_grid.filter (*cloud);

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  // Save the aligned template for visualization
  pcl::PointCloud<PointMap::Point> transformed_cloud;
  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);

  return (0);
}


#endif
