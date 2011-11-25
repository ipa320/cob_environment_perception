/*
 * ppm_to_pcd.cpp
 *
 *  Created on: 18.07.2011
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

vector<string> file_i(2, "");
string file_o = "";


void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in", value< vector<string> >(&file_i), "input files, first ppm, second pcd")
    ("out", value<string> (&file_o), "output pcd file")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 2).add("out", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << options << endl;
    exit(0);
  }
  if (file_o == "") 
  {
    cout << "no output file defined " << endl << options << endl;
    exit(0);
  }
  if (file_i[0] == "" || file_i[1] == "") 
  {
    cout << "no input files defined " << endl << options << endl;
    exit(0);
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZ>::Ptr p(new PointCloud<PointXYZ>());
  PointCloud<PointXYZRGBA>::Ptr pc(new PointCloud<PointXYZRGBA>());

  PCDReader r;
  if (r.read(file_i[1], *p) == -1) return(0);
  copyPointCloud<PointXYZ, PointXYZRGBA>(*p, *pc);
  pc->height = p->height;
  pc->width = p->width;
  cob_3d_mapping_tools::PPMReader ppmR;
  if (ppmR.mapRGB(file_i[0], *pc) == -1)
  {
    cout << "Mapping error" << endl;
    return(0);
  }
  cout << "Mapped colors to \"" << file_o << "\" (Points: " << pc->points.size() << ", width: " 
       << pc->width << ", height: " << pc->height << ")" << endl;
  PCDWriter w;
  io::savePCDFileASCII(file_o, *pc);
  return(0);
}
