/*
 * pcd_to_ppm.cpp
 *
 *  Created on: 07.07.2011
 *      Author: goa-sf
 */

// Boost:
#include <boost/program_options.hpp>
// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <cob_3d_mapping_tools/io.h>

using namespace std;
using namespace pcl;

vector<string> file_o(2, "");
string file_i = "";

float min_z, max_z;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in", value<string>(&file_i), "input pcd file")
    ("out", value< vector<string> >(&file_o), "output files, first rgb, [second depth]")
    ("min,m", value<float>(&min_z)->default_value(FLT_MAX), "min value of depth range")
    ("max,M", value<float>(&max_z)->default_value(FLT_MIN), "max value of depth range")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1).add("out", 2);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << options << endl;
    exit(0);
  }
  if (file_i == "") 
  {
    cout << "no input and output file defined " << endl << options << endl;
    exit(0);
  }
  if (file_o[0] == "") 
  {
    cout << "no output file defined " << endl << options << endl;
    exit(0);
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGBA>::Ptr pc(new PointCloud<PointXYZRGBA>());
  PointCloud<PointXYZRGB>::Ptr p_(new PointCloud<PointXYZRGB>());
  cout << "has_denorm: " << numeric_limits<float>::has_denorm << endl;
  cout << "denormalized: " << numeric_limits<float>::denorm_min() << endl;
  PCDReader r;
  if (r.read(file_i, *p_) == -1) 
  {
    if (r.read(file_i, *pc) == -1) return(0);
  }
  else
  {
    pc->resize(p_->size());
    pc->width = p_->width;
    pc->height = p_->height;
    for(size_t i = 0; i < p_->size(); ++i)
    {
//      if (p_->points[i].rgb < numeric_limits<float>::min()) 
//	cout << (double)p_->points[i].rgb << endl;
      pc->points[i].x = p_->points[i].x;
      pc->points[i].y = p_->points[i].y;
      pc->points[i].z = p_->points[i].z;
      pc->points[i].rgba = p_->points[i].rgba;
    }
    
  }
  
  cout << "Loaded pointcloud with " << pc->width << " x " << pc->height << " points." << endl;

  cob_3d_mapping_tools::PPMWriter w;
  if (w.writeRGB(file_o[0], *pc) == -1) return(0);
  cout << "Extracted RGB image..." << endl;

  if (file_o[1] != "")
  {
    if (min_z != FLT_MAX)
      w.setMinZ(min_z);
    if (max_z != FLT_MIN)
      w.setMaxZ(max_z);

    if (w.writeDepth(file_o[1], *pc) == -1) return(0);
    cout << "Extracted depth image..." << endl;
  }

  cout << "Done!" << endl;

  return(0);
}
