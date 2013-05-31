
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>

#include "cob_3d_mapping_filters/downsample_filter.h"
#include "cob_3d_mapping_features/organized_normal_estimation_omp.h"
#include "cob_3d_segmentation/impl/fast_segmentation.hpp"
#include "cob_3d_segmentation/polygon_extraction/polygon_extraction.h"



namespace cob_3d_segmentation
{
  class MeshSegmentation
  {
  public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    typedef pcl::PointCloud<PointLabel> LabelCloud;
    typedef cob_3d_segmentation::FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel>::ClusterPtr ClusterPtr;


  public:
    MeshSegmentation()
      : one_()
      , seg_()
      , down_(new PointCloud)
      , segmented_(new PointCloud)
      , normals_(new NormalCloud)
      , labels_(new LabelCloud)
      {
        one_.setOutputLabels(labels_);
        one_.setPixelSearchRadius(8,2,2);
        one_.setSkipDistantPointThreshold(8);

        seg_.setNormalCloudIn(normals_);
        seg_.setLabelCloudInOut(labels_);
        seg_.setSeedMethod(SEED_RANDOM);

      }

    void performSegmentation(const PointCloud::ConstPtr& input)
    {
      cob_3d_mapping_filters::DownsampleFilter<pcl::PointXYZRGB> down;
      down_->header = input->header;
      down.setInputCloud(input);
      down.filter(*down_);
      *segmented_ = *down_;

      one_.setInputCloud(down_);
      one_.compute(*normals_);

      seg_.setInputCloud(down_);
      seg_.compute();
      seg_.mapSegmentColor(segmented_);
    }

    cob_3d_mapping_features::OrganizedNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> one_;
    cob_3d_segmentation::FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel, cob_3d_segmentation::Options::ComputeGraph> seg_;
    PolygonExtraction pe_;

    PointCloud::Ptr down_;
    PointCloud::Ptr segmented_;
    NormalCloud::Ptr normals_;
    LabelCloud::Ptr labels_;
  };
}

std::string file_in_;
std::string file_out_;

int readOptions(int argc, char** argv)
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help,h", "produce help message")
    ("in,i", value<std::string>(&file_in_), "input folder with data points")
    ("out,o", value<std::string>(&file_out_), "out file with data points")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if(vm.count("help") || argc == 1)
  { std::cout << options << std::endl; return(-1); }
}


int main(int argc, char** argv)
{
  readOptions(argc, argv);

  pcl::PCDReader r;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGB>);
  r.read(file_in_, *p);

  cob_3d_segmentation::MeshSegmentation mseg;
  return 0;
}
