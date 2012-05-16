// OpenCV:
#include <opencv2/core/core.hpp>
#include <cv.h>
#include <highgui.h>

#include <boost/program_options.hpp>
// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// Packages Includes:
#include "cob_3d_mapping_features/edge_estimation_3d.h"
#include "cob_3d_mapping_features/edge_estimation_2d.h"
#include "cob_3d_mapping_common/point_types.h"

using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

string file_points;
float normal_radius;
float edge_th;
bool rgbEdges, depthEdges;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<string>(&file_points), "input pcd file")
    ("normal_radius,R", value<float>(&normal_radius)->default_value(0.01),
     "radius normal estimation")
    ("edgeth,e",value<float>(&edge_th)->default_value(0.5), "threshold edge")
    ("rgbedges,c", value<bool>(&rgbEdges)->default_value(false), "calculate color edges")
    ("depthedges,d", value<bool>(&depthEdges)->default_value(false), "calculate range image edges")
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
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<InterestPoint>::Ptr ip(new PointCloud<InterestPoint>);
  PointCloud<InterestPoint>::Ptr ip2(new PointCloud<InterestPoint>);

  PCDReader r;
  if(r.read(file_points, *p) == -1) return(0);
      //for electric
    //KdTreeFLANN<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>);
    //for fuerte
   pcl::search::KdTree<PointXYZRGB>::Ptr tree(new  pcl::search::KdTree<PointXYZRGB>);

  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setRadiusSearch(normal_radius);
  ne.setSearchMethod(tree);
  ne.setInputCloud(p);
  ne.compute(*n);
  //for electric
  //OrganizedDataIndex<PointXYZRGB>::Ptr oTree (new OrganizedDataIndex<PointXYZRGB> );
  //for fuerte
  search::OrganizedNeighbor<PointXYZRGB>::Ptr oTree (new search::OrganizedNeighbor<PointXYZRGB> );
  cob_3d_mapping_features::EdgeEstimation3D<PointXYZRGB, Normal, InterestPoint> ee;
  ee.setRadiusSearch(0.04);
  ee.setSearchMethod(oTree);
  ee.setInputCloud(p);
  ee.setInputNormals(n);
  ee.dist_threshold_ = 0.02;
  ee.compute(*ip);
  cout << "Done!" << endl;

  if (rgbEdges)
  {
    cout << "Start 2D part" << endl;
    cv::Mat sobel, laplace, combined, color;
    cob_3d_mapping_features::EdgeEstimation2D<PointXYZRGB, InterestPoint> ee2;
    ee2.setInputCloud(p);
    ee2.getColorImage(color);
    ee2.computeEdges(*ip2);
    ee2.computeEdges(sobel, laplace, combined);
    cv::imshow("Color", color);
    cv::imshow("Combined", combined);
    cv::imshow("Laplace", laplace);
    cv::imshow("Sobel", sobel);
    cv::waitKey();
  }

  if (depthEdges)
  {
    cout << "Start depth part" << endl;
    cv::Mat sobel, laplace, combined, range;
    cob_3d_mapping_features::EdgeEstimation2D<PointXYZRGB, InterestPoint> ee3;
    ee3.setInputCloud(p);
    ee3.getRangeImage(range, 0.0, 0.0);
    ee3.computeEdgesFromRange(sobel, laplace, combined);
    cv::imshow("Range", range);
    cv::imshow("Combined", combined);
    cv::imshow("Laplace", laplace);
    cv::imshow("Sobel", sobel);
    cv::waitKey();
  }

  for (size_t i = 0; i < ip2->points.size(); i++)
  {
    /*int color = max(0.0f, min(ip2->points[i].strength*255, 255.0f) );
    p->points[i].r = color;
    p->points[i].g = color;
    p->points[i].b = color;*/

    if (ip->points[i].strength >= 0.3 && ip->points[i].strength <1.0)
    {
      p->points[i].r = 255;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
    /*else if (ip->points[i].strength > edge_th)
    {
      p->points[i].r = 0;
      p->points[i].g = 255;
      p->points[i].b = 0;
    }*/
    else
    {
      p->points[i].r = 0;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
  }

  boost::shared_ptr<visualization::PCLVisualizer> v;
  v.reset(new visualization::PCLVisualizer(file_points));
  v->setBackgroundColor(0,127,127);
  ColorHdlRGB col_hdl1(p);
  v->addPointCloud<PointXYZRGB>(p, col_hdl1, "edge");

  while(!v->wasStopped())
  {
    v->spinOnce(100);
    usleep(100000);
  }
  return(0);
}
