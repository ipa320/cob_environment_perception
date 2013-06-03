
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>

#include <cob_3d_mapping_common/stop_watch.h>
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

    inline PointCloud::Ptr getCloudDown() { return down_; }
    inline PointCloud::Ptr getCloudSegmented() { return segmented_; }
    inline NormalCloud::Ptr getCloudNormals() { return normals_; }
    inline LabelCloud::Ptr getCloudLabels() { return labels_; }


    void performSegmentation(const PointCloud::ConstPtr& input)
    {
      PrecisionStopWatch t;
      t.precisionStart();
      cob_3d_mapping_filters::DownsampleFilter<pcl::PointXYZRGB> down;
      down_->header = input->header;
      down.setInputCloud(input);
      down.filter(*down_);
      *segmented_ = *down_;
      std::cout << "downsampling took " << t.precisionStop() << " s." << std::endl;

      t.precisionStart();
      one_.setInputCloud(down_);
      one_.compute(*normals_);
      std::cout << "normals took " << t.precisionStop() << " s." << std::endl;

      t.precisionStart();
      seg_.setInputCloud(down_);
      seg_.compute();
      seg_.mapSegmentColor(segmented_);
      std::cout << "segmentation took " << t.precisionStop() << " s." << std::endl;

      int w = labels_->width;
      int mask[] = { -w, 1, w, -1 };
      int curr_label, count;
      Eigen::Vector3f curr_p;
      for (size_t y = w; y < labels_->size() - w; y+=w)
      {
        for (size_t i=y+1; i < y+w-1; ++i)
        {
          curr_label = (*labels_)[i].label;
          if(curr_label == I_NAN)
          {
            //(*down_)[i].rgb = (*segmented_)[i].rgb;
            (*down_)[i].r = 1.0f;
            (*down_)[i].g = 1.0f;
            (*down_)[i].b = 1.0f;
            continue;
          }
          curr_p = (*down_)[i].getVector3fMap();
          count = 0;
          for (int m=0; m<4; ++m)
          {
            if (curr_label != (*labels_)[ i+mask[m] ].label &&
                !cob_3d_mapping::PrimeSense::areNeighbors((*down_)[ i+mask[m] ].getVector3fMap(), curr_p))
            {
              ++count;
            }
          }
          if(count >= 4 || count < 1)
          {
            (*down_)[i].r = 0.0f;
            (*down_)[i].g = 0.0f;
            (*down_)[i].b = 0.0f;
          }
          else
          {
            (*down_)[i].rgb = (*segmented_)[i].rgb;
          }
        }
      }
    }

    cob_3d_mapping_features::OrganizedNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> one_;
    cob_3d_segmentation::FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel,
                                          cob_3d_segmentation::Options::ComputeGraph> seg_;
    //cob_3d_segmentation::FastSegmentation<pcl::PointXYZRGB, pcl::Normal, PointLabel> seg_;
    PolygonExtraction pe_;

    PointCloud::Ptr down_;
    PointCloud::Ptr segmented_;
    NormalCloud::Ptr normals_;
    LabelCloud::Ptr labels_;
  };
}

std::string file_in_;
std::string file_out_;
bool save_full_image_;

int readOptions(int argc, char** argv)
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help,h", "produce help message")
    ("in,i", value<std::string>(&file_in_), "input folder with data points")
    ("out,o", value<std::string>(&file_out_), "out file with data points")
    ("dense,d", "save dense segmentation image")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1).add("out",1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if(vm.count("help") || argc == 1)
  { std::cout << options << std::endl; return(-1); }
  if(vm.count("dense")) save_full_image_ = true;
  else save_full_image_ = false;
}


int main(int argc, char** argv)
{
  readOptions(argc, argv);

  pcl::PCDReader r;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGB>);
  r.read(file_in_, *p);

  cob_3d_segmentation::MeshSegmentation mseg;
  mseg.performSegmentation(p);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr res;
  res = mseg.getCloudDown();

  pcl::PCDWriter w;
  if(save_full_image_) w.write(file_out_, *mseg.getCloudSegmented());
  else w.write(file_out_, *res);
  return 0;
}
