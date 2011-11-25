/*
 * Separate feature values (RSD,PC,FPFH) of a classified point cloud 
 *
 */

#include <cob_3d_mapping_common/label_defines.h>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

string in_label;
string in_feature;
string out;

bool plane;
bool edge;
bool cyl;
bool sph;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in_label,l", value<string>(&in_label)->default_value(""),
     "pcd file containting rgba labeled points")
    ("in_feature,f", value<string>(&in_feature)->default_value(""),
     "pcd file containting feature values")
    ("out,o", value<string>(&out)->default_value(""),
     "output folder")
    ("plane,p", value<bool>(&plane)->default_value(true), 
     "extract points labeled as plane")
    ("edge,e", value<bool>(&edge)->default_value(true), 
     "extract points labeled as edge")
    ("cyl,c", value<bool>(&cyl)->default_value(true), 
     "extract points labeled as cylinder")
    ("sph,s", value<bool>(&sph)->default_value(true), 
     "extract points labeled as sphere")
    ;

  positional_options_description p_opt;
  p_opt.add("out", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << options << endl;
    exit(0);
  }
}

template<typename PointT> 
void extractIndices(const string & file_in, 
		    const string & folder, 
		    const string & prefix,
		    const vector< vector<int> > & indices)
{
  PointCloud<PointT> p;
  PointCloud<PointT> p_ex;

  io::loadPCDFile<PointT>(file_in, p);
  if (indices[0].size() != 0)
  {
    copyPointCloud<PointT>(p, indices[0], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_plane.pcd", p_ex);
  }
  if (indices[1].size() != 0)
  {
    copyPointCloud<PointT>(p, indices[1], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_edge.pcd", p_ex);
  }
  if (indices[2].size() != 0)
  {
    copyPointCloud<PointT>(p, indices[2], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_cyl.pcd", p_ex);
  }
  if (indices[3].size() != 0)
  {
    copyPointCloud<PointT>(p, indices[3], p_ex);
    io::savePCDFileASCII<PointT>(folder + prefix + "_sph.pcd", p_ex);
  }
}


int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGBA>::Ptr p_label (new PointCloud<PointXYZRGBA>);
  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);

  vector<int> idx_plane;
  vector<int> idx_edge;
  vector<int> idx_cyl;
  vector<int> idx_sph;

  vector< vector<int> > indices(4, vector<int>());
  int mode = 0;

  io::loadPCDFile(in_label, *p_label);
  io::loadPCDFile(in_feature, *cloud);
  if (getFieldIndex(*cloud, "r_max") != -1)
  {
    cout << "Point cloud with RSD features selected." << endl;
    mode = 1;
  }
  else if (getFieldIndex(*cloud, "pc1") != -1)
  {
    cout << "Point cloud with principal curvatures selected." << endl;
    mode = 2;
  }
  else if (getFieldIndex(*cloud, "fpfh") != -1)
  {
    cout << "Point cloud with FPFH selected." << endl;
    mode = 3;
  }

  for (size_t i = 0; i < p_label->size(); ++i)
  {
    switch(p_label->points[i].rgba)
    {
    case LBL_PLANE:
      indices[0].push_back(i);
      break;

    case LBL_EDGE:
      indices[1].push_back(i);
      break;

    case LBL_CYL:
      indices[2].push_back(i);
      break;

    case LBL_SPH:
      indices[3].push_back(i);
      break;

    default:
      break;
    }
  }

  if(!plane)
    indices[0].clear();
  if(!edge)
    indices[1].clear();
  if(!cyl)
    indices[2].clear();
  if(!sph)
    indices[3].clear();

  switch(mode)
  {
  case 1:
    extractIndices<PrincipalRadiiRSD>(in_feature, out, "rsd", indices);
    break;

  case 2:
    extractIndices<PrincipalCurvatures>(in_feature, out, "pc", indices);
    break;

  case 3:
    extractIndices<FPFHSignature33>(in_feature, out, "fpfh", indices);
    break;

  default:
    break;
  }

  
  return 1;
}
