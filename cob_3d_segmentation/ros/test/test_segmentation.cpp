/*
 * test_segmentation.cpp
 *
 *  Created on: 18.06.2012
 *      Author: josh
 */


//includes needed for testing
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

//includes needed for segmentation (things to test)
#include <cob_3d_segmentation/general_segmentation.h>
#include <cob_3d_segmentation/quad_regression/quad_regression.h>

/**
 * little helper to load pcd files from test folder
 */
class Testing_PCDLoader
{
  std::vector<sensor_msgs::PointCloud2> pc2s_;

  void load(const std::string &fn) {
    sensor_msgs::PointCloud2 pc2;
    if(!pcl::io::loadPCDFile(fn,pc2))
      pc2s_.push_back(pc2);
    else
      ROS_ERROR("failed to load pcd file %s",fn.c_str());
  }

  Testing_PCDLoader()
  {
    load("test/kitchen01_close_raw.pcd");
    load("test/kitchen01_far_raw.pcd");
    load("test/office01_close_raw.pcd");
    load("test/office01_far_raw.pcd");
    load("test/shelves01_close_raw.pcd");
    load("test/shelves01_far_raw.pcd");
    load("test/table01_close1m_raw.pcd");
    load("test/table01_far_raw.pcd");
    //TODO: rest
  }

public:
  static Testing_PCDLoader &get() {
    static Testing_PCDLoader t;
    return t;
  }

  template <typename Point>
  bool getPC(const size_t ind, typename pcl::PointCloud<Point>::Ptr pc) const
  {
    if(ind<pc2s_.size())
    {
      pcl::fromROSMsg(pc2s_[ind],*pc);
      return true;
    }
    return false;
  }
};

/**
 * stopping time precise
 */

class PrecisionStopWatch {
  struct timeval precisionClock;

public:
  PrecisionStopWatch() {
    gettimeofday(&precisionClock, NULL);
  };

  void precisionStart() {
    gettimeofday(&precisionClock, NULL);
  };

  double precisionStop() {
    struct timeval precisionClockEnd;
    gettimeofday(&precisionClockEnd, NULL);
    double d= ((double)precisionClockEnd.tv_sec + 1.0e-6 * (double)precisionClockEnd.tv_usec) - ((double)precisionClock.tv_sec + 1.0e-6 * (double)precisionClock.tv_usec);
    precisionStart();
    return d;
  };
};



template <typename Point, typename PointLabel>
void segment_pointcloud(GeneralSegmentation<Point, PointLabel> *seg, typename pcl::PointCloud<Point>::Ptr &pc)
{
  EXPECT_TRUE(seg!=NULL);
  EXPECT_TRUE(pc->size()>0);

  seg->setInputCloud(pc);

  PrecisionStopWatch psw;
  EXPECT_TRUE(
      seg->compute()
      );
  ROS_INFO("segmentation took %f", psw.precisionStop());
}

TEST(Segmentation, quad_regression)
{
  typedef pcl::PointXYZ Point;
  typedef pcl::PointXYZ PointL;

  pcl::PointCloud<Point>::Ptr pc(new pcl::PointCloud<Point>);
  Segmentation::Segmentation_QuadRegression<Point,PointL> seg;

  ROS_INFO("starting segmentation");
  size_t ind=0;
  while(Testing_PCDLoader::get().getPC<Point>(ind++, pc))
  {
    ROS_INFO("processing pc %d ...",(int)ind-1);
    segment_pointcloud<Point,PointL>(&seg,pc);
  }
}


int main(int argc, char **argv){
  ros::Time::init();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
