#include "cob_3d_mapping_features/segmentation.h"
#include <pcl/io/pcd_io.h>
#include <highgui.h>

int main()
{
  Segmentation seg;
  pcl::PointCloud<PointLabel>::Ptr cloud_in = pcl::PointCloud<PointLabel>::Ptr (new pcl::PointCloud<PointLabel>);
  cv::Mat seg_img;
  std::vector<pcl::PointIndices> cluster_indices;
  /*cloud_in->points.resize(900);
  cloud_in->width = 30;
  cloud_in->height = 30;
  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(i<14 && j==14) cloud_in->points[i*30+j].label = 1;
      if(i>16 && j==14) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg",seg_img);

  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(j<14 && i==14) cloud_in->points[i*30+j].label = 1;
      if(j>16 && i==14) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg2",seg_img);

  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(i<14 && j==14) cloud_in->points[i*30+j].label = 1;
      if(i>16 && j==15) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg3",seg_img);

  for(unsigned int i=0; i<30; i++)
  {
    for(unsigned int j=0; j<30; j++)
    {
      cloud_in->points[i*30+j].label = 0;
      if(i<14 && j==14) cloud_in->points[i*30+j].label = 1;
    }
  }
  //pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  seg.propagateWavefront2(cloud_in);
  seg.getClusterIndices(cloud_in, cluster_indices, seg_img);
  cv::imshow("seg4",seg_img);*/

  pcl::io::loadPCDFile("/home/goa/pcl_daten/corner/output/edge_cloud.pcd",*cloud_in);
  pcl::PointCloud<PointLabel>::Ptr cloud_in_c = pcl::PointCloud<PointLabel>::Ptr (new pcl::PointCloud<PointLabel>);
  for(unsigned int i=0; i<cloud_in->points.size(); i++)
  {
    if(i>=64000)
    {
      cloud_in_c->points.push_back(cloud_in->points[i]);
    }
  }
  cloud_in_c->width = 640;
  cloud_in_c->height = 380;
  seg.propagateWavefront2(cloud_in_c);
  seg.getClusterIndices(cloud_in_c, cluster_indices, seg_img);
  cv::imshow("seg_real",seg_img);

  cv::waitKey();
  return 0;
}
