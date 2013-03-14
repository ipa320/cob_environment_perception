/*
 * test_segmentation.cpp
 *
 *  Created on: 18.06.2012
 *      Author: josh
 */

//#define USE_COLOR

//includes needed for testing
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <rosbag/bag.h>

//includes needed for segmentation (things to test)
#include <cob_3d_segmentation/general_segmentation.h>
#include <cob_3d_segmentation/quad_regression/quad_regression.h>
#include <cob_3d_segmentation/ransac/ransac.h>
#include <cob_3d_segmentation/marching_cubes/marching_cubes.h>
#ifdef PCL_MINOR_VERSION >= 6
#include <cob_3d_segmentation/multi_plane/multi_plane.h>
#endif


struct PointXYZRGBLabel
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  union
  {
    union
    {
      struct
      {
        uint8_t b;
        uint8_t g;
        uint8_t r;
        uint8_t _unused;
      };
      float rgb;
    };
    uint32_t rgba;
  };
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBLabel,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (uint32_t, label, label));

/**
 * little helper to load pcd files from test folder
 */
class Testing_PCDLoader
{
  std::vector<std::string> pc2s_fn_;

  static bool _load(const std::string &fn, sensor_msgs::PointCloud2 &pc2) {
    pc2.header.frame_id = fn; ///filename is stored in frame_id

    if(!pcl::io::loadPCDFile(fn,pc2))
      return true;
    else
      ROS_ERROR("failed to load pcd file %s",fn.c_str());

    return false;
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
  }

public:

  /// singleton
  static Testing_PCDLoader &get() {
    static Testing_PCDLoader t;
    return t;
  }

  void load(const std::string &fn) {
    pc2s_fn_.push_back(fn);
  }

  template <typename Point>
  bool getPC(const size_t ind, typename pcl::PointCloud<Point>::Ptr pc, std::string &fn) const
  {
    pc->clear();
    if(ind<pc2s_fn_.size())
    {
      sensor_msgs::PointCloud2 pc2;
      if(_load(pc2s_fn_[ind],pc2))
      {
      pcl::fromROSMsg(pc2,*pc);
      fn = pc2.header.frame_id;
      }
      return true;
    }
    return false;
  }

  template <typename Point>
  void writePC(const std::string &pc_fn, typename pcl::PointCloud<Point>::ConstPtr pc) const
  {
    const ::testing::TestInfo* const test_info =
      ::testing::UnitTest::GetInstance()->current_test_info();

    if(pc->size()<1) {
	ROS_ERROR("cannot save empty pc (%s_%s_%s)", test_info->test_case_name(), test_info->name(), pc_fn.c_str());
	return;
    }

    char fn[512];
    sprintf(fn,"test/labeled/pc_%s_%s_%s",test_info->test_case_name(), test_info->name(), pc_fn.c_str());
    pcl::io::savePCDFile(fn,*pc);
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

/**
 * write down a table
 */
class Testing_CSV
{
  FILE *fp;
  int row, col;
public:
  Testing_CSV(std::string fn):row(0),col(0)
  {
    const ::testing::TestInfo* const test_info =
      ::testing::UnitTest::GetInstance()->current_test_info();
    fn = "test/results/csv_"+std::string(test_info->test_case_name())+"_"+std::string(test_info->name())+"_"+fn+".csv";
    fp = fopen(fn.c_str(),"w");
  }

  ~Testing_CSV()
  {
    if(fp) fclose(fp);
  }

  void add(const std::string &str)
  {
    if(col>0) fputc('\t',fp);
    fwrite(str.c_str(),str.size(),1,fp);
    ++col;
  }

  template<typename T>
  void add(const T v)
  {
    std::stringstream ss(std::stringstream::out);
    ss<<v;
    add(ss.str());
  }

  void next()
  {
    if(col>0)
    {
      fputc('\n',fp);
      col=0;
      ++row;
    }
    fflush(fp);
  }

  static Testing_CSV create_table(const std::string &fn, std::string cols)
  {
    Testing_CSV csv(fn);
    size_t pos;
    while(cols.size()>0)
    {
      pos=cols.find(",");
      if(pos!=std::string::npos)
      {
        csv.add(std::string(cols.begin(),cols.begin()+pos));
        cols.erase(cols.begin(), cols.begin()+(pos+1));
      }
      else
      {
        csv.add(cols);
        break;
      }
    }
    csv.next();
    return csv;
  }
};




/*******************************TESTING STARTS HERE********************************/

template <typename Point, typename PointLabel>
void segment_pointcloud(GeneralSegmentation<Point, PointLabel> *seg, typename pcl::PointCloud<Point>::Ptr &pc, const std::string &fn)
{
  EXPECT_TRUE(seg!=NULL);
  EXPECT_TRUE(pc->size()>0);

  //for documentation
  static Testing_CSV csv = Testing_CSV::create_table("execution_time","filename,execution time in seconds");
  double took=0.;

  seg->setInputCloud(pc);

  PrecisionStopWatch psw;
  EXPECT_TRUE(
      seg->compute()
      );
  ROS_INFO("segmentation took %f", took=psw.precisionStop());
  csv.add(fn);
  csv.add(took);
  csv.next();

  Testing_PCDLoader::get().writePC<PointLabel>(fn, seg->getOutputCloud());
}

template <int Degree>
void test_QPPF()
{
  typedef pcl::PointXYZ Point;
  typedef PointXYZRGBLabel PointL;

  pcl::PointCloud<Point>::Ptr pc(new pcl::PointCloud<Point>);
  static Segmentation::Segmentation_QuadRegression<Point, PointL, Segmentation::QPPF::QuadRegression<Degree, Point, Segmentation::QPPF::CameraModel_Kinect<Point> > > seg;

  static Testing_CSV csv = Testing_CSV::create_table("accuracy","filename,mean,variance,weighted mean,weighted variance,average distance,used points, memory for representation,points,true positive rate, false positive rate,execution time: quadtree,execution time: growing,execution time: extraction,number of segments");

  ROS_INFO("starting segmentation");
  size_t ind=0;
  std::string fn;
  while(Testing_PCDLoader::get().getPC<Point>(ind++, pc, fn))
  {
    if(pc->size()<1) continue;

    pcl::PointCloud<PointL>::Ptr labeled_pc(new pcl::PointCloud<PointL>);
    bool loaded = false;
    try {
      loaded = Testing_PCDLoader::get().getPC<PointL>(ind-1, labeled_pc, fn);
    } catch(...) {
    }
    if(!loaded) {
	ROS_INFO("will not evaluate segmentation");
	labeled_pc.reset();}

    ROS_INFO("processing pc %d ...",(int)ind-1);
    std::string fn_short(fn.begin()+(fn.find_last_of("/")+1),fn.end());

    //for(size_t i=0; i<pc->size(); i++) if(pcl_isfinite((*pc)[i].z)) (*pc)[i].z = (*pc)[i].x*(*pc)[i].y;

    segment_pointcloud<Point,PointL>(&seg,pc, fn_short);

    seg.extractImages();

    Testing_PCDLoader::get().writePC<PointL>("reconstructed_"+fn_short, seg.getReconstructedOutputCloud());

    float mean, var, mean_weighted, var_weighted, dist;
    double et_quadtree, et_growing, et_extraction;
    double true_positive, false_positive;
    size_t used, mem, points;
    seg.compute_accuracy(mean, var, mean_weighted, var_weighted, used, mem, points, dist, labeled_pc, true_positive, false_positive);
#ifdef STOP_TIME
    seg.getExecutionTimes(et_quadtree, et_growing, et_extraction);
#endif

    csv.add(fn_short);
    csv.add(mean);
    csv.add(var_weighted);
    csv.add(mean_weighted);
    csv.add(var);
    csv.add(dist);
    csv.add(used);
    csv.add(mem);
    csv.add(points);
    csv.add(true_positive);
    csv.add(false_positive);
#ifdef STOP_TIME
    csv.add(et_quadtree);
    csv.add(et_growing);
    csv.add(et_extraction);
#endif
    csv.add(seg.getPolygons().size());
    csv.next();

    //saving ros msgs to bag-file
    rosbag::Bag bag_out;
    bag_out.open("test/labeled/"+fn_short+".bag", rosbag::bagmode::Write);
    bag_out.write("shapes_array", ros::Time(1342850029.582334425+0.1), (cob_3d_mapping_msgs::ShapeArray)seg);
    bag_out.write("shapes_array", ros::Time(1342850029.582334425+0.2), (cob_3d_mapping_msgs::ShapeArray)seg);
    bag_out.write("shapes_array", ros::Time(1342850029.582334425+0.4), (cob_3d_mapping_msgs::ShapeArray)seg);
  }
}

//TEST(Segmentation, quad_regression1)
//{ test_QPPF<1>(); }
//TEST(Segmentation, quad_regression2)
//{ test_QPPF<2>(); }
//TEST(Segmentation, quad_regression3)
//{ test_QPPF<3>(); }
//TEST(Segmentation, quad_regression4)
//{ test_QPPF<4>(); }
//TEST(Segmentation, quad_regression6)
//{ test_QPPF<6>(); }

//degree 10 didn't work on my nb

TEST(Segmentation, ransac)
{
  return;
  typedef pcl::PointXYZRGB Point;
  typedef PointXYZRGBLabel PointL;

  pcl::PointCloud<Point>::Ptr pc(new pcl::PointCloud<Point>);
  Segmentation::Segmentation_RANSAC<Point,PointL> seg;

  static Testing_CSV csv = Testing_CSV::create_table("accuracy","filename,mean,variance,average distance,used points, memory for representation,points,true positive rate, false positive rate");

  ROS_INFO("starting segmentation");
  size_t ind=0;
  std::string fn;
  while(Testing_PCDLoader::get().getPC<Point>(ind++, pc, fn))
  {
    if(pc->size()<1) continue;
    ROS_INFO("processing pc %d ...",(int)ind-1);
    std::string fn_short(fn.begin()+(fn.find_last_of("/")+1),fn.end());
    segment_pointcloud<Point,PointL>(&seg,pc, fn_short);

    pcl::PointCloud<PointL>::Ptr labeled_pc(new pcl::PointCloud<PointL>);
    try {
      Testing_PCDLoader::get().getPC<PointL>(ind-1, labeled_pc, fn);
    } catch(...) {}

    Testing_PCDLoader::get().writePC<PointL>("reconstructed_"+fn_short, seg.getReconstructedOutputCloud());

    float mean, var, mean_weighted, var_weighted, dist;
    double true_positive, false_positive;
    size_t used, mem, points;
    seg.compute_accuracy(mean, var, mean_weighted, var_weighted, used, mem, points, dist, labeled_pc, true_positive, false_positive);

    csv.add(fn_short);
    csv.add(mean);
    csv.add(var);
    csv.add(dist);
    csv.add(used);
    csv.add(mem);
    csv.add(points);
    csv.add(true_positive);
    csv.add(false_positive);
    csv.next();
  }
}

#ifdef PCL_MINOR_VERSION >= 6
TEST(Segmentation, multi_plane)
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointXYZRGBNormal PointN;
  typedef PointXYZRGBLabel PointL;

  pcl::PointCloud<Point>::Ptr pc(new pcl::PointCloud<Point>);
  Segmentation::Segmentation_MultiPlaneExtraction<Point,PointN,PointL> seg;

  static Testing_CSV csv = Testing_CSV::create_table("accuracy","filename,mean,variance,average distance,used points, memory for representation,points,true positive rate, false positive rate");

  ROS_INFO("starting segmentation");
  size_t ind=0;
  std::string fn;
  while(Testing_PCDLoader::get().getPC<Point>(ind++, pc, fn))
  {
    if(pc->size()<1) continue;
    ROS_INFO("processing pc %d ...",(int)ind-1);
    std::string fn_short(fn.begin()+(fn.find_last_of("/")+1),fn.end());
    segment_pointcloud<Point,PointL>(&seg,pc, fn_short);

    pcl::PointCloud<PointL>::Ptr labeled_pc(new pcl::PointCloud<PointL>);
    try {
      Testing_PCDLoader::get().getPC<PointL>(ind-1, labeled_pc, fn);
    } catch(...) {}

    Testing_PCDLoader::get().writePC<PointL>("reconstructed_"+fn_short, seg.getReconstructedOutputCloud());

    float mean, var, mean_weighted, var_weighted, dist;
    double true_positive, false_positive;
    size_t used, mem, points;
    seg.compute_accuracy(mean, var, mean_weighted, var_weighted, used, mem, points, dist, labeled_pc, true_positive, false_positive);

    csv.add(fn_short);
    csv.add(mean);
    csv.add(var);
    csv.add(dist);
    csv.add(used);
    csv.add(mem);
    csv.add(points);
    csv.add(true_positive);
    csv.add(false_positive);
    csv.next();
  }
}
#endif

TEST(Segmentation, marching_cubes)
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointXYZRGBNormal PointN;
  typedef pcl::PointXYZRGB PointL;

  pcl::PointCloud<Point>::Ptr pc(new pcl::PointCloud<Point>);
  Segmentation::Segmentation_MarchingCubes<Point,PointN,PointL> seg;

  static Testing_CSV csv = Testing_CSV::create_table("accuracy","filename,mean,variance,average distance,used points, memory for representation,points");

  ROS_INFO("starting segmentation");
  size_t ind=0;
  std::string fn;
  while(Testing_PCDLoader::get().getPC<Point>(ind++, pc, fn))
  {
    if(pc->size()<1) continue;
    ROS_INFO("processing pc %d ...",(int)ind-1);
    std::string fn_short(fn.begin()+(fn.find_last_of("/")+1),fn.end());
    segment_pointcloud<Point,PointL>(&seg,pc, fn_short);

    Testing_PCDLoader::get().writePC<PointL>("reconstructed_"+fn_short, seg.getReconstructedOutputCloud());

    float mean, var, dist;
    size_t used, mem, points;
    seg.compute_accuracy(mean, var, used, mem, points, dist);

    csv.add(fn_short);
    csv.add(mean);
    csv.add(var);
    csv.add(dist);
    csv.add(used);
    csv.add(mem);
    csv.add(points);
    csv.next();
  }
}


int main(int argc, char **argv){
  ros::Time::init();

  if(argc>1 && strcmp(argv[1],"--help")==0)
  {
    printf("usage:\n\t- without any options tests segmentation\n\t- with additional pcd files\n");
    return 0;
  }

  for(int i=1; i<argc; i++)
    Testing_PCDLoader::get().load(argv[i]);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
