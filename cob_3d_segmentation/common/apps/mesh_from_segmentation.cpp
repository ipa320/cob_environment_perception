
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


    int what(int idx, int* mask, int mm)
    {
      int curr_label;
      int next_label = (*labels_)[idx].label;
      int res = 0;
      for(int i = 0; i<mm; ++i)
      {
        curr_label = next_label;
        next_label = (*labels_)[idx + mask[i]].label;
        res |= (curr_label != next_label) << i; // flip i-th bit if not equal
      }
      return res;
    }

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
      int mask[] = { 1, 1+w, w, 0};
      int update[16];
      update[0] = update[1] = update[2] = update[4] = update[8] = 0; // nothing
      update[3] = update[6] = update[9] = update[12] = 4; // 2 clusters (3+1)
      update[5] = update[10] = 1; // 2 clusters (2+2)
      update[7] = update[11] = update[13] = update[14] = update[15] = 16; // 3 and 4 clusters
      borders_.reset(new LabelCloud);
      borders_->resize(labels_->size());
      for (size_t y = 0; y < labels_->size() - w; y+=w)
      {
        for (size_t i=y; i < y+w-1; ++i)
        {
          int res = update[what(i, mask, 4)];
          (*borders_)[ i + mask[1] ].label = res;
          (*borders_)[ i + mask[2] ].label += res;
          (*borders_)[ i + mask[3] ].label += res;
          (*borders_)[ i + mask[0] ].label += res;
        }
      }

      for (int i=0; i<borders_->size(); ++i)
      {
        switch( (*borders_)[i].label )
        {
        case 2: // straight
        { (*down_)[i].rgba = LBL_CYL; break; } //green

        case 5: // 45°/90° outer corner
        { (*down_)[i].rgba = LBL_EDGE_CVX; break; } //orange

        case 6: // 90° inner corner
        { (*down_)[i].rgba = LBL_EDGE; break; } //red

        case 9: // begin/end of curve
        { (*down_)[i].rgba = LBL_BORDER; break; } //magenta

        case 12: // 45° diagonal
        { (*down_)[i].rgba = LBL_SPH_CVX; break; } //purple

        default:
        {
          if( (*borders_)[i].label > 16 ) // edge point
          {
            (*down_)[i].rgba = LBL_SPH; //blue
          }
          else // nothing
          {
            (*down_)[i].rgba = LBL_NAN;
          }
          break;
        }
        }
      }

      /*
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
      */
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
    LabelCloud::Ptr borders_;
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
