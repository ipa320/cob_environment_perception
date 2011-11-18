#include <boost/program_options.hpp>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>

#include <cob_3d_mapping_common/label_defines.h>

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

  PointCloud<FPFHSignature33> plane, e_vex, e_cav, c_vex, c_cav, s_vex, s_cav, tmp, all; 
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
      else if (string::npos != files[i].rfind("cylinder_convex_"))
	c_vex += tmp;
      else if (string::npos != files[i].rfind("cylinder_concave_"))
	c_cav += tmp;
      tmp.clear();
    }
  }

  all += plane;
  all += e_vex;
  all += e_cav;
  all += c_vex;
  all += c_cav;
  all += s_vex;
  all += s_cav;
  PCDWriter w;
  w.write(outfolder_ + "knn_fpfh_features.pcd", all);

  string labels_file_name;
  labels_file_name = outfolder_ + "knn_labels.txt";
  ofstream f (labels_file_name.c_str());
  f << "CS" << endl;
  for(size_t i = 0; i < plane.size(); ++i)
    f << SVM_PLANE << endl;
  for(size_t i = 0; i < e_vex.size(); ++i)
    f << SVM_EDGE_CVX << endl;
  for(size_t i = 0; i < e_cav.size(); ++i)
    f << SVM_EDGE_CAV << endl;
  for(size_t i = 0; i < c_vex.size(); ++i)
    f << SVM_CYL_CVX << endl;
  for(size_t i = 0; i < c_cav.size(); ++i)
    f << SVM_CYL_CAV << endl;
  for(size_t i = 0; i < s_vex.size(); ++i)
    f << SVM_SPH_CVX << endl;
  for(size_t i = 0; i < s_cav.size(); ++i)
    f << SVM_SPH_CAV << endl;
  f.close();

  return 1;

}
