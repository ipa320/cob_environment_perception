// Boost:
#include <boost/program_options.hpp>
#include <boost/timer.hpp>

// PCL:
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include "cob_3d_mapping_common/stop_watch.h"
#include <cob_3d_mapping_tools/impl/sav_gol_smoothing_filter.hpp>

template class SavGolSmoothingFilter<pcl::PointXYZRGB, pcl::PointXYZRGB>;

using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;


string file_, file_out_ ="";
float th_;
int th2_;
int size_, poly_, size2nd_, poly2nd_;
bool vis_, mls_;


void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<string>(&file_), "input pcd file")
    ("out,o", value<string>(&file_out_), "save the smoothed cloud if you want")
    ("threshold_modifier,t", value<float>(&th_)->default_value(4.0), "thresholdmodifier")
    ("other_threshold_modifier,T", value<int>(&th2_)->default_value(7), "other thresholdmodifier")
    ("size,s", value<int>(&size_)->default_value(33), "size")
    ("poly,p", value<int>(&poly_)->default_value(6), "order of polynomial")
    ("size2nd,S", value<int>(&size2nd_)->default_value(11), "size for second pass")
    ("poly2nd,P", value<int>(&poly2nd_)->default_value(4), "order of polynomial for second pass")
    ("vis,v", value<bool>(&vis_)->default_value(false), "enable visualization")
    ("mls,m", value<bool>(&mls_)->default_value(false), "enable mls smoothing")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << options << endl;
    exit(0);
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr p2(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr out(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr out2(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr out3(new PointCloud<PointXYZRGB>);

  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);
  *p2 = *p;
  PrecisionStopWatch t;
  if (mls_)
  {
    t.precisionStart();
    // MLS
    #ifdef PCL_VERSION_COMPARE //fuerte
      pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>());
    #else //electric
      pcl::KdTreeFLANN<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB> ());
    #endif
    tree->setInputCloud(p);
    MovingLeastSquares<PointXYZRGB, Normal> mls;
    mls.setInputCloud(p);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(4);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    mls.reconstruct(*out3);
    cout << t.precisionStop() << "s\t for MLS" << endl;
  }


  // SavGol on default depth data
  t.precisionStart();
  vector<size_t> indices;
  SavGolSmoothingFilter<pcl::PointXYZRGB, pcl::PointXYZRGB> savgol;
  savgol.setInputCloud(p);
  savgol.setFilterCoefficients(size_,poly_);
  savgol.setDistanceThreshold(0.05);
  savgol.reconstruct(*out, indices);
  cout << t.precisionStop() << "s\t for SavGol on depth data" << endl;

  t.precisionStart();
  // SavGol on dispartiy data
  for (size_t i = 0; i<p->size(); i++)
  {
    p->points[i].z = round(1090.0f - ( 345.0f / p->points[i].z ));
  }
  indices.clear();
  savgol.setInputCloud(p);
  savgol.setFilterCoefficients(size_,poly_);
  savgol.setDistanceThreshold(th2_);
  savgol.reconstruct(*out2, indices);

  savgol.setFilterCoefficients(size2nd_,poly2nd_);
  savgol.setDistanceThreshold(th2_);
  savgol.reconstruct2ndPass(indices, *out2);
  for (size_t i = 0; i<p->size(); i++)
  {
    out2->points[i].z = 345.0f / (1090.0f - out2->points[i].z );
  }
  cout << t.precisionStop() << "s\t for SavGol on disparity data" << endl;

  if (file_out_ != "")
  {
    io::savePCDFileASCII<PointXYZRGB>(file_out_, *out2);
  }

  if (vis_)
  {
    visualization::PCLVisualizer v;
    ColorHdlRGB col_hdl(p);

    /* --- Viewports: ---
     *  1y 
     *    | 1 | 3 |
     * .5 ----+----
     *    | 2 | 4 |
     *  0    .5    1x
     * 1:
     */
    // xmin, ymin, xmax, ymax
    int v1(0);
    v.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
    v.setBackgroundColor(0,127,127, v1);
    v.addPointCloud<PointXYZRGB>(p2,col_hdl, "org", v1);
    //v.addPointCloudNormals<PointXYZRGB, Normal>(p,n,10,0.04,"normals1", v1);

    int v2(0);
    v.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    v.setBackgroundColor(0,127,127, v2);
    v.addPointCloud<PointXYZRGB>(out,col_hdl, "savgol1", v2);
    //v.addPointCloudNormals<PointXYZRGB, Normal>(p,n,10,0.04,"normals2", v2);

    int v3(0);
    v.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
    v.setBackgroundColor(0,127,127, v3);
    v.addPointCloud<PointXYZRGB>(out2,col_hdl, "savgol2", v3);

    int v4(0);
    v.createViewPort(0.5, 0.0, 1.0, 0.5, v4);
    v.setBackgroundColor(0,127,127, v4);
    v.addPointCloud<PointXYZRGB>(out3,col_hdl, "mls", v4);

    while(!v.wasStopped())
    {
      v.spinOnce(100);
      usleep(100000);
    }
  }
  return(0);
}
