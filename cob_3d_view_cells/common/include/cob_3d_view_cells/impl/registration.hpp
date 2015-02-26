#pragma once

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <cob_3d_visualization/simple_marker.h>
struct DebugRegistration {
	ros::Publisher pub_A_, pub_B_;
	
	DebugRegistration(ros::NodeHandle *n) {
		pub_A_ = n->advertise<sensor_msgs::PointCloud2>("dbg_reg_A", 1);
		pub_B_ = n->advertise<sensor_msgs::PointCloud2>("dbg_reg_B", 1);
		cob_3d_visualization::RvizMarkerManager::get().createTopic("view_cells_debug_markers").setFrameId("/camera_depth_optical_frame").clearOld();
	}
	
	void publish(const pcl::PointCloud<pcl::PointXYZ> &A, pcl::PointCloud<pcl::PointXYZ> B, const pcl::Correspondences &cors) {
		cob_3d_visualization::RvizMarkerManager::get().clear();
		
		sensor_msgs::PointCloud2 AA,BB;
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		transform(2,3)+=2;
		pcl::transformPointCloud (B,B, transform);
		pcl::toROSMsg(A,AA);
		pcl::toROSMsg(B,BB);
		AA.header.frame_id = BB.header.frame_id = "/camera_rgb_optical_frame";
		pub_A_.publish(AA);
		pub_B_.publish(BB);
		
		for(size_t i=0; i<A.size(); i++) {
			cob_3d_visualization::RvizMarker arrow;
			arrow.arrow<Eigen::Vector3f>(A[i].getVector3fMap(), B[i].getVector3fMap(), 0.01f);
			arrow.color(1,0,0,0.1);
		}
		for(size_t i=0; i<cors.size(); i++) {
			cob_3d_visualization::RvizMarker arrow;
			arrow.arrow<Eigen::Vector3f>(A[cors[i].index_query].getVector3fMap(), B[cors[i].index_match].getVector3fMap(), 0.02f);
		}
		cob_3d_visualization::RvizMarkerManager::get().publish();
	}
};

static DebugRegistration *g_dbg_reg = NULL;

Eigen::Matrix4f VRegistration::computeTF(bool *result) const
{  
  double sac_threshold = 0.3;
  int sac_max_iterations = 100;
  pcl::Correspondences correspondeces_sac;
  pcl::registration::CorrespondenceRejectorSampleConsensus<Point> cor_rej_sac;
  cor_rej_sac.setInputSource(pcA_);
  cor_rej_sac.setInputTarget(pcB_);
  cor_rej_sac.setInlierThreshold(sac_threshold);
  cor_rej_sac.setMaximumIterations(sac_max_iterations);
  cor_rej_sac.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondeces_));
  cor_rej_sac.getCorrespondences(correspondeces_sac);
  
  if(result) *result = correspondeces_sac.size()>5;// *correspondeces_sac.size()>(correspondeces_.size()+5);

  printf("%d\t correspondences after SAC rejection (%d)\n", (int)correspondeces_sac.size(), (int)correspondeces_.size());

  Eigen::Matrix4f transform_sac = cor_rej_sac.getBestTransformation();

  std::vector<int> indices_not_corresponding;
  cor_rej_sac.getRejectedQueryIndices(correspondeces_sac, indices_not_corresponding);
  printf("%d\t correspondences rejected by SAC\n", (int)indices_not_corresponding.size());


#if 0
  pcl::registration::TransformationEstimationSVD<Point, Point> trans_est;
  Eigen::Matrix4f transform_svd;
  trans_est.estimateRigidTransformation(*pcA_, *pcB_, correspondeces_sac, transform_svd);
#else
  pcl::IterativeClosestPointNonLinear<Point, Point> reg;
  reg.setTransformationEpsilon (1e-6);
  
  //if(std::abs(transform_sac(2,3))>0.3f) transform_sac = Eigen::Matrix4f::Identity();
  if(std::abs(transform_sac(2,3))>0.3f) transform_sac.topLeftCorner<3,3>() = Eigen::Matrix3f::Identity();
  if(transform_sac(2,3)>0.3f) transform_sac(2,3)=0.3f;
  else if(transform_sac(2,3)<-0.3f) transform_sac(2,3)=-0.3f;
  
  reg.setMaxCorrespondenceDistance (0.25);  
  //reg.setPointRepresentation (boost::make_shared<const Point> (point_representation));

  reg.setInputSource (pcA_);
  reg.setInputTarget (pcB_);
  
  reg.setMaximumIterations (50);
  
  pcl::PointCloud<Point> aligned_pc;
  reg.align (aligned_pc, transform_sac);
  Eigen::Matrix4f transform_svd = reg.getFinalTransformation ();
#endif

  std::cout << "Transform from SAC:" << std::endl;
  std::cout << transform_sac << std::endl;
  std::cout << "Transform from SVD:" << std::endl;
  std::cout << transform_svd << std::endl;
  
  if(g_dbg_reg) g_dbg_reg->publish(*pcA_,*pcB_,correspondeces_sac);
  
  if(result) *result &= ( std::abs(transform_svd(2,3))<0.3f && transform_svd.col(3).head(3).squaredNorm()<0.8f*0.8f && transform_svd(0,0)>0.8f && transform_svd!=Eigen::Matrix4f::Identity() );
  
  return transform_svd;
}
