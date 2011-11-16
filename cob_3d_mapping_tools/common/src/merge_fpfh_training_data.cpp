#include <boost/program_options.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>

//#include <cob_3d_features/svm_class_defines.h>

using namespace std;
using namespace pcl;

string outfolder_;
vector<string> folder_in_;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  
  options_description cmd_line("Options");
  cmd_line.add_options()
    ("help", "produce help messsage")
    ("out,o", value<string>(&outfolder_), "output folder")
    ("in,i", value<vector<string> >(&folder_in_), "in files")
    ;

  positional_options_description p_opt;
  p_opt.add("in", -1);

  variables_map vm;
  store(command_line_parser(argc, argv)
	.options(cmd_line)
	.positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("in"))
  {
    cout << cmd_line << endl;
    exit(0);
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<FPFHSignature33> plane, e_vex, e_cav, c_vex, c_cav, s_vex, s_cav, tmp;
  PCDReader r;
  for (size_t j = 0; j < folder_in_.size(); j++)
  {
    vector<string> files;
    getAllPcdFilesInDirectory(folder_in_[j], files);
    for (size_t i = 0; i < files.size(); i++)
    {
      files[i] = folder_in_[j] + files[i];
      r.read(files[i], tmp);
      if(string::npos != files[i].rfind("plane"))
	plane += tmp;
      else if (string::npos != files[i].rfind("edge_convex_"))
	e_vex += tmp;
      else if (string::npos != files[i].rfind("edge_concave_"))
	e_cav += tmp;
      else if (string::npos != files[i].rfind("sphere_convex_"))
	s_vex += tmp;
      else if (string::npos != files[i].rfind("sphere_concave_"))
	s_cav += tmp;	
      else if (string::npos != files[i].rfind("cylinder_convex"))
	c_vex += tmp;
      else if (string::npos != files[i].rfind("cylinder_concave"))
	c_cav += tmp;
      tmp.clear();
    }
  }

  PCDWriter w;
  w.write(outfolder_ + "fpfh_plane_.pcd", plane);
  w.write(outfolder_ + "fpfh_edge_convex_.pcd", e_vex);
  w.write(outfolder_ + "fpfh_cylinder_convex_.pcd", c_vex);
  w.write(outfolder_ + "fpfh_sphere_convex_.pcd", s_vex);
  w.write(outfolder_ + "fpfh_edge_concave_.pcd", e_cav);
  w.write(outfolder_ + "fpfh_cylinder_concave_.pcd", c_cav);
  w.write(outfolder_ + "fpfh_sphere_concave_.pcd", s_cav);

  return 1;
}
