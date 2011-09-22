/*
 * point_map.cpp
 *
 * Created on: Sep 6, 2011
 * Author: goa-jh
 */

#include "cob_env_model/table_object_cluster.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <cob_env_model_msgs/GetFieldOfView.h>
#include <Eigen/Core>
#include "cob_env_model/point_map.h"


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

#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/statistical_outlier_removal.h>

class TestCondition2 : public pcl::ConditionOr<PointMap::Point> {
public:
  virtual bool evaluate (const PointMap::Point &point) const {
    ROS_INFO("%f\n",point.x);
    return point.x<-3;//&&point.x>-3.3;
  }
};

void PointMap::transform(const pcl::PointCloud<Point>::Ptr& pc_in, const pcl::PointCloud<Point>::Ptr& pc, const StampedTransform &transform) {
  //transformPointCloud("/map", transform, pc->header.stamp, *(pc.get()), *(pc.get()));
  //pcl_ros::transformPointCloud ("/map", *(pc.get()), *(pc.get()), tf_listener_);
  pcl_ros::transformPointCloud(*pc, *pc, transform);
  pcl_ros::transformPointCloud(*pc_in, *pc_in, transform);
  pc->header.frame_id = "/map";
  pc_in->header.frame_id = "/map";
  //shiftCloud(pc);
  //shiftCloud(pc_in);
}

bool PointMap::compute(const pcl::PointCloud<Point>::Ptr& pc_in, const pcl::PointCloud<Point>::Ptr& pc, const StampedTransform &transform, const cob_env_model_msgs::GetFieldOfView *get_fov_srv) {
  boost::timer t;


  pcl::PointCloud<Point> pc_aligned;
  Eigen::Matrix4f icp_transform;
  bool ret=false;
  if(use_reference_map_)
    ret = doFOVICPUsingReference(pc, pc_aligned, icp_transform);
  else
    ret = doFOVICP(pc, pc_aligned, icp_transform, get_fov_srv);
  if(ret)
  {
    std::cout << "icp_transform: " << icp_transform << std::endl;
    transform_old_ = transform;
    old_icp_transform_ = icp_transform;
    pcl::transformPointCloud(*pc_in,*pc_in,icp_transform);
    pc_aligned.header.frame_id = "/map";
    map_.header.frame_id="/map";
    if(first_)
    {
      pcl::copyPointCloud(pc_aligned, map_);
      map_.header.frame_id="/map";
      first_ = false;
    }
    else
      map_ += pc_aligned;
    //map_ += *pc_in;
    //doICP(pc);
  }
  else
    ROS_INFO("FOV not successful");

  compution_time_=t.elapsed();

  ROS_INFO("[point_map] ICP took %f s", t.elapsed());

  return ret;
}

bool PointMap::doFOVICP(const pcl::PointCloud<Point>::Ptr& pc,
                        pcl::PointCloud<Point>& pc_aligned,
                        Eigen::Matrix4f& final_transformation, const cob_env_model_msgs::GetFieldOfView *get_fov_srv)
{

  if(first_)
  {
    pcl::copyPointCloud(*pc, pc_aligned);
    final_transformation = Eigen::Matrix4f::Identity();
    return true;
  }

  if(!get_fov_srv)
    return false;

  n_up_t_(0) = get_fov_srv->response.fov.points[0].x;
  n_up_t_(1) = get_fov_srv->response.fov.points[0].y;
  n_up_t_(2) = get_fov_srv->response.fov.points[0].z;
  n_down_t_(0) = get_fov_srv->response.fov.points[1].x;
  n_down_t_(1) = get_fov_srv->response.fov.points[1].y;
  n_down_t_(2) = get_fov_srv->response.fov.points[1].z;
  n_right_t_(0) = get_fov_srv->response.fov.points[2].x;
  n_right_t_(1) = get_fov_srv->response.fov.points[2].y;
  n_right_t_(2) = get_fov_srv->response.fov.points[2].z;
  n_left_t_(0) = get_fov_srv->response.fov.points[3].x;
  n_left_t_(1) = get_fov_srv->response.fov.points[3].y;
  n_left_t_(2) = get_fov_srv->response.fov.points[3].z;
  n_origin_t_(0) = get_fov_srv->response.fov.points[4].x;
  n_origin_t_(1) = get_fov_srv->response.fov.points[4].y;
  n_origin_t_(2) = get_fov_srv->response.fov.points[4].z;
  n_max_range_t_(0) = get_fov_srv->response.fov.points[5].x;
  n_max_range_t_(1) = get_fov_srv->response.fov.points[5].y;
  n_max_range_t_(2) = get_fov_srv->response.fov.points[5].z;


  //segment FOV
  seg_.setInputCloud(map_.makeShared());
  //transformNormals(map_.header.frame_id, pc->header.stamp);
  pcl::PointIndices indices;
  seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, n_origin_t_, n_max_range_t_);
  pcl::PointCloud<Point> frustum;
  pcl::ExtractIndices<Point> extractIndices;
  extractIndices.setInputCloud(map_.makeShared());
  extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  extractIndices.filter(frustum);
  ROS_DEBUG("[aggregate_point_map] Frustum size: %d", (int)frustum.size());

  /*TODO:
   * if (save_map_fov_==true)
{
std::stringstream ss;
ss << file_path_ << "/map_fov_" << ctr_ << ".pcd";
pcl::io::savePCDFileASCII (ss.str(), frustum);
}*/

  //do ICP
  pcl::IterativeClosestPoint<Point,Point> icp;
  icp.setInputCloud(pc->makeShared());
  //icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  icp.setInputTarget(frustum.makeShared());
  icp.setMaximumIterations(icp_max_iterations_);
  icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
  icp.setTransformationEpsilon (icp_trf_epsilon_);
  //pcl::PointCloud<Point> pc_aligned;
  icp.align(pc_aligned);
  final_transformation = icp.getFinalTransformation();

  //TODO: check if converged, check fitness score; if not => don't use pc
  ROS_INFO("[aggregate_point_map] ICP has converged: %d\n", icp.hasConverged());
  ROS_INFO("[aggregate_point_map] Fitness score: %f", icp.getFitnessScore());
  return true;
}

class MyIterativeClosestPoint : public pcl::IterativeClosestPoint<PointMap::Point,PointMap::Point> {
public:
  int getNeededIterations() {return nr_iterations_;}
};

class TestCondition : public pcl::ConditionOr<PointMap::Point> {
  PointMap::Point center;
public:
  TestCondition(PointMap::Point center):center(center)
  {}

  virtual bool evaluate (const PointMap::Point &point) const {
    PointMap::Point p;
    p.x = point.x-center.x;
    p.y = point.y-center.y;
    p.z = point.z-center.z;
    float lq=p.x*p.x + p.y*p.y + p.z*p.z;

    return lq<1.5;
  }
};

#include <pcl/Vertices.h>
#include "cob_env_model/features/plane_extraction.h"
#include "cob_env_model_msgs/PlaneExtractionAction.h"
#include <vector>

void extractPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in,
                  std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > >& v_cloud_hull,
                  std::vector<std::vector<pcl::Vertices> >& v_hull_polygons,
                  std::vector<pcl::ModelCoefficients>& v_coefficients_plane)
{
  PlaneExtraction pe;
  pe.extractPlanes(pc_in, v_cloud_hull, v_hull_polygons, v_coefficients_plane);
}

bool PointMap::doFOVICPUsingReference(const pcl::PointCloud<Point>::Ptr& pc,
                                      pcl::PointCloud<Point>& pc_aligned,
                                      Eigen::Matrix4f& final_transformation)
{
  pcl::PointCloud<Point> pc2;


    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > v_cloud_hull;
    std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
    std::vector<pcl::ModelCoefficients> v_coefficients_plane;
    extractPlane(pc->makeShared(), v_cloud_hull, v_hull_polygons, v_coefficients_plane);
    for(unsigned int i = 0; i < v_cloud_hull.size(); i++)
    {
      float o=0,g=0;
      int n=0;
      Eigen::Vector3f v;
      for(int j=0; j<v_cloud_hull[i].size()-2; j++) {
        Eigen::Vector3f v1(v_cloud_hull[i][j+0].x, v_cloud_hull[i][j+0].y, v_cloud_hull[i][j+0].z);
        Eigen::Vector3f v2(v_cloud_hull[i][j+1].x, v_cloud_hull[i][j+1].y, v_cloud_hull[i][j+1].z);
        Eigen::Vector3f v3(v_cloud_hull[i][j+2].x, v_cloud_hull[i][j+2].y, v_cloud_hull[i][j+2].z),a,b;
        a=v2-v1;
        b=v3-v2;
        v+=v1;
        a.normalize();
        b.normalize();
        ++n;
        float f=a.dot(b);
        //g+=cosf(f);
        g+=v1(0)*v1(0) + v1(1)*v1(1) + v1(2)*v1(2);
        if(o*f<0) {
          //ROS_INFO("%d dot",n);
          n=0;
        }
        o=f;
      }

      Eigen::Vector4f min_pt, max_pt;
      pcl::PointXYZ center;
      pcl::getMinMax3D(v_cloud_hull[i], min_pt, max_pt);
      center.x = (min_pt(0)+max_pt(0))/2;
      center.y = (min_pt(1)+max_pt(1))/2;
      center.z = (min_pt(2)+max_pt(2))/2;

      pcl::PointXYZ a,b;
      a.x=min_pt(0);
      a.y=min_pt(1);
      a.z=min_pt(2);
      b.x=max_pt(0);
      b.y=max_pt(1);
      b.z=max_pt(2);
      /*pc2.insert(pc2.end(),a);
      pc2.insert(pc2.end(),b);
      pc2.insert(pc2.end(),center);*/

      pc2+=v_cloud_hull[i];

      v/=v_cloud_hull[i].size()-2;
      ROS_INFO("%f %f %f %f pos",center.x,center.y,center.z, (max_pt-min_pt).norm());//v(0),v(1),v(2));
      ROS_INFO("%f ges",g);
      //ROS_INFO("%d points\n",v_cloud_hull[i].size());
      //ROS_INFO("%d planes published so far", i);
    }

  {
    ROS_INFO("%d %d LLLLL",pc2.size(), ref_map_.size());

  //do ICP
  MyIterativeClosestPoint icp;
  icp.setInputCloud(pc2.makeShared());
  //icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  icp.setInputTarget(ref_map_.makeShared());
  icp.setMaximumIterations(icp_max_iterations_);
  icp.setRANSACOutlierRejectionThreshold(0.8);
  if(first_)
    icp.setMaxCorrespondenceDistance(icp_max_corr_dist_on_first_);
  else
    icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
  icp.setTransformationEpsilon (icp_trf_epsilon_);

  //icp.align(pc_aligned, old_icp_transform_);
  if(first_||!use_reuse_)
    icp.align(pc_aligned);
  else
    icp.align(pc_aligned, old_icp_transform_);

  final_transformation = icp.getFinalTransformation();

  pc_aligned.clear();
  for(unsigned int i = 0; i < v_cloud_hull.size(); i++) {
    pcl::transformPointCloud(v_cloud_hull[i],v_cloud_hull[i], final_transformation);
    pc_aligned+=v_cloud_hull[i];
  }

  ROS_INFO("[aggregate_point_map] ICP iterations: %d\n", icp.getNeededIterations());

  return icp.getMaximumIterations()!=icp.getNeededIterations()&&icp.getNeededIterations()>0;
  }
  /*cob_env_model::GetFieldOfView get_fov_srv;
get_fov_srv->request.target_frame = std::string("/map");
get_fov_srv->request.stamp = pc->header.stamp;
if(get_fov_srv_client_.call(get_fov_srv))
{
ROS_DEBUG("FOV service called [OK].");
}
else
{
ROS_WARN("FOV service called [FAILED].");
return false;
}
n_up_t_(0) = get_fov_srv->response.fov.points[0].x;
n_up_t_(1) = get_fov_srv->response.fov.points[0].y;
n_up_t_(2) = get_fov_srv->response.fov.points[0].z;
n_down_t_(0) = get_fov_srv->response.fov.points[1].x;
n_down_t_(1) = get_fov_srv->response.fov.points[1].y;
n_down_t_(2) = get_fov_srv->response.fov.points[1].z;
n_right_t_(0) = get_fov_srv->response.fov.points[2].x;
n_right_t_(1) = get_fov_srv->response.fov.points[2].y;
n_right_t_(2) = get_fov_srv->response.fov.points[2].z;
n_left_t_(0) = get_fov_srv->response.fov.points[3].x;
n_left_t_(1) = get_fov_srv->response.fov.points[3].y;
n_left_t_(2) = get_fov_srv->response.fov.points[3].z;
n_origin_t_(0) = get_fov_srv->response.fov.points[4].x;
n_origin_t_(1) = get_fov_srv->response.fov.points[4].y;
n_origin_t_(2) = get_fov_srv->response.fov.points[4].z;
n_max_range_t_(0) = get_fov_srv->response.fov.points[5].x;
n_max_range_t_(1) = get_fov_srv->response.fov.points[5].y;
n_max_range_t_(2) = get_fov_srv->response.fov.points[5].z;


//segment FOV
seg_.setInputCloud(ref_map_.makeShared());Call the regist
//transformNormals(map_.header.frame_id, pc->header.stamp);
pcl::PointIndices indices;
seg_.segment(indices, n_up_t_, n_down_t_, n_right_t_, n_left_t_, n_origin_t_, n_max_range_t_);
pcl::PointCloud<Point> frustum;
pcl::ExtractIndices<Point> extractIndices;
extractIndices.setInputCloud(ref_map_.makeShared());
extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
extractIndices.filter(frustum);
ROS_DEBUG("[aggregate_point_map] Frustum size: %d", (int)frustum.size());
if (save_map_fov_==true)
{
std::stringstream ss;
ss << file_path_ << "/map_fov_" << ctr_ << ".pcd";
pcl::io::savePCDFileASCII (ss.str(), frustum);
}*/
  /*ROS_INFO("[EPSILON] %f\n",icp_trf_epsilon_);

  pcl::SampleConsensusInitialAlignment<Point, Point, Point> sac;
  sac.setInputCloud(pc->makeShared());
  //sac.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  sac.setInputTarget(ref_map_.makeShared());
  sac.setMinSampleDistance (0.05);
  sac.setMaximumIterations(icp_max_iterations_);
  if(first_)
    sac.setMaxCorrespondenceDistance(icp_max_corr_dist_on_first_);
  else
    sac.setMaxCorrespondenceDistance(icp_max_corr_dist_);
  sac.setTransformationEpsilon (icp_trf_epsilon_);
  ROS_INFO("[EPSILON] %f\n",icp_trf_epsilon_);
  //icp.align(pc_aligned, old_icp_transform_);
  if(first_||!use_reuse_)
      sac.align(pc_aligned);
  else
      sac.align(pc_aligned, old_icp_transform_);

  old_icp_transform_ = sac.getFinalTransformation();
  first_=false;*/
  /*
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*pc, min_pt, max_pt);
  pcl::PointXYZ center;
  center.x = (min_pt(0)+max_pt(0))/2;
  center.y = (min_pt(1)+max_pt(1))/2;
  center.z = (min_pt(2)+max_pt(2))/2;

  pcl::PointCloud<Point>::Ptr pc2 = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);


  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (pc);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1);
  //sor.setNegative(true);
  sor.filter (*pc);
/*
  pcl::ConditionalRemoval<Point> cr;
  cr.setInputCloud(pc);
  //cr.setKeepOrganized(true);
  cr.setCondition(pcl::ConditionalRemoval<Point>::ConditionBasePtr(new TestCondition(center)));
  cr.filter(*pc2);*/

  //do ICP
  MyIterativeClosestPoint icp;
  icp.setInputCloud(pc->makeShared());
  //icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  icp.setInputTarget(ref_map_.makeShared());
  icp.setMaximumIterations(icp_max_iterations_);
  icp.setRANSACOutlierRejectionThreshold(0.3);
  if(first_)
    icp.setMaxCorrespondenceDistance(icp_max_corr_dist_on_first_);
  else
    icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
  icp.setTransformationEpsilon (icp_trf_epsilon_);
  //pcl::PointCloud<Point> pc_aligned;

  /*std::vector<pcl::PointCloud<Point>, Eigen::aligned_allocator<pcl::PointCloud<Point> > > v_cloud_hull;
pcl::PointCloud< std::string > tt,tt2;
tt.insert(tt.begin(),std::string("hi"));
std::string key="test";
icp.setSourceFeature< std::string >(tt.makeShared(),key);
icp.setTargetFeature< std::string >(tt2.makeShared(),key);*/

  //icp.align(pc_aligned, old_icp_transform_);
  if(first_||!use_reuse_)
    icp.align(pc_aligned);
  else
    icp.align(pc_aligned, old_icp_transform_);

  final_transformation = icp.getFinalTransformation();

  //std::cout<<tt2.size();

#if POINT_MAP_FITNESS_TEST
  fitness_=icp.getFitnessScore(10.);

  //TODO: check if converged, check fitness score; if not => don't use pc
  ROS_INFO("[aggregate_point_map] ICP params (max_it, corr_dist, eps): %d,%f,%f\n", icp_max_iterations_, icp_max_corr_dist_, icp_trf_epsilon_);
  ROS_INFO("[aggregate_point_map] ICP has converged: %d\n", icp.hasConverged());
  ROS_INFO("[aggregate_point_map] Fitness score: %f", icp.getFitnessScore());
#endif

  ROS_INFO("[aggregate_point_map] ICP iterations: %d\n", icp.getNeededIterations());

  return icp.getMaximumIterations()!=icp.getNeededIterations()&&icp.getNeededIterations()>0;
}


void PointMap::doICP(const pcl::PointCloud<Point>::Ptr& pc)
{
  //TODO: change map2_ to map_, flag for IO operations, publish to callback
  //Open file for timer log
  boost::timer t;

  //Perform ICP
  pcl::IterativeClosestPoint<Point,Point> icp;
  icp.setInputCloud(pc);
  icp.setInputTarget(map_.makeShared());
  icp.setMaximumIterations(icp_max_iterations_);
  icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
  icp.setTransformationEpsilon (icp_trf_epsilon_);
  pcl::PointCloud<Point> pc_aligned;
  icp.align(pc_aligned);
  map_ += pc_aligned;

  //TODO: output as ROS_DEBUG
  double time = t.elapsed();
  ROS_DEBUG("ICP has converged: %d\n", icp.hasConverged());
  ROS_DEBUG("Fitness score: %f", icp.getFitnessScore());
  ROS_DEBUG("\tTime: %f", time);

  //TODO: parameter for file path
  if(save_icp_map_==true)
  {
    std::stringstream ss1;
    ss1 << file_path_ << *pctr_ << ".pcd";
    pcl::io::savePCDFileASCII (ss1.str(), map_);
  }
  /*pcl::VoxelGrid<Point> vox_filter;
vox_filter.setInputCloud(map_.makeShared());
vox_filter.setLeafSize(0.03, 0.03, 0.03);
vox_filter.filter(map2_);*/
  //point_cloud_pub_.publish(map2_);
}

void PointMap::clearMap()
{
  //TODO: add mutex
  map_.points.clear();
  map_.width = 0;
  map_.height = 0;
}

bool PointMap::setReferenceMap(cob_env_model_msgs::SetReferenceMap::Request &req)
{
  pcl::PointCloud<Point> rmap;
  pcl::fromROSMsg(req.map, rmap);
  setReferenceMap(rmap);
  return true;
}

void PointMap::setReferenceMap(pcl::PointCloud<Point> &rmap)
{
  ref_map_.clear();

  std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > v_cloud_hull;
  std::vector<std::vector<pcl::Vertices> > v_hull_polygons;
  std::vector<pcl::ModelCoefficients> v_coefficients_plane;
  extractPlane(rmap.makeShared(), v_cloud_hull, v_hull_polygons, v_coefficients_plane);
  for(unsigned int i = 0; i < v_cloud_hull.size(); i++)
  {
    float o=0,g=0;
    int n=0;
    Eigen::Vector3f v;
    for(int j=0; j<v_cloud_hull[i].size()-2; j++) {
      Eigen::Vector3f v1(v_cloud_hull[i][j+0].x, v_cloud_hull[i][j+0].y, v_cloud_hull[i][j+0].z);
      Eigen::Vector3f v2(v_cloud_hull[i][j+1].x, v_cloud_hull[i][j+1].y, v_cloud_hull[i][j+1].z);
      Eigen::Vector3f v3(v_cloud_hull[i][j+2].x, v_cloud_hull[i][j+2].y, v_cloud_hull[i][j+2].z),a,b;
      a=v2-v1;
      b=v3-v2;
      v+=v1;
      a.normalize();
      b.normalize();
      ++n;
      float f=a.dot(b);
      //g+=cosf(f);
      g+=v1(0)*v1(0) + v1(1)*v1(1) + v1(2)*v1(2);
      if(o*f<0) {
        //ROS_INFO("%d dot",n);
        n=0;
      }
      o=f;
    }

    Eigen::Vector4f min_pt, max_pt;
    pcl::PointXYZ center;
    pcl::getMinMax3D(v_cloud_hull[i], min_pt, max_pt);
    center.x = (min_pt(0)+max_pt(0))/2;
    center.y = (min_pt(1)+max_pt(1))/2;
    center.z = (min_pt(2)+max_pt(2))/2;

    //ref_map_.insert(ref_map_.end(),center);
    ref_map_+=v_cloud_hull[i];

    v/=v_cloud_hull[i].size()-2;
    ROS_INFO("%f %f %f %f pos",center.x,center.y,center.z, (max_pt-min_pt).norm());//v(0),v(1),v(2));
    ROS_INFO("%f ges",g);
    //ROS_INFO("%d points\n",v_cloud_hull[i].size());
    //ROS_INFO("%d planes published so far", i);
  }
}

void PointMap::shiftCloud(const pcl::PointCloud<Point>::Ptr& pc)
{
  for(unsigned int i=0; i<pc->size(); i++)
    pc->points[i].y+=0.15;
  //pc->points[i].z+=0.2;
}
