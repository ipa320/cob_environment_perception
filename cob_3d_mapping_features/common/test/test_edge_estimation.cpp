#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <cob_3d_mapping_features/edge_estimation.h>
#include <cob_3d_mapping_common/point_types.h>

using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

string file_points;
float normal_radius;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in,i", value<string>(&file_points), "input pcd file")
    ("normal_radius,R", value<float>(&normal_radius)->default_value(0.01), 
     "radius normal estimation")
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

  PCDReader r;
  if(r.read(file_points, *p) == -1) return(0);

  KdTreeFLANN<PointXYZRGB>::Ptr tree(new KdTreeFLANN<PointXYZRGB>);
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setRadiusSearch(normal_radius);
  ne.setSearchMethod(tree);
  ne.setInputCloud(p);
  ne.compute(*n);

  OrganizedDataIndex<pcl::PointXYZRGB>::Ptr oTree (new OrganizedDataIndex<pcl::PointXYZRGB> );
  ipa_features::EdgeEstimation<PointXYZRGB, Normal, InterestPoint> ee;
  ee.setRadiusSearch(0.04);
  ee.setSearchMethod(oTree);
  ee.setInputCloud(p);
  ee.setInputNormals(n);
  ee.dist_threshold_ = 0.02;
  ee.compute(*ip);
  cout << "Done!" << endl;

  for (size_t i = 0; i < ip->points.size(); i++)
  {
    int color = max(0.0f, min(ip->points[i].strength*255, 255.0f) );
    
    p->points[i].r = color;
    p->points[i].g = color;
    p->points[i].b = color;
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
