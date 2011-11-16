#include <cob_3d_mapping_tools/point_generator.h>
#include <cob_3d_mapping_common/label_defines.h>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <math.h>

using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> ColorHdlRGBA;
typedef Eigen::Vector3f Vec;

string file;
bool visualize;
float noise;
float corner_size;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("out,o", value<string>(&file)->default_value(""),
     "output file, set to \"\" for no output")
    ("noise,n", value<float>(&noise)->default_value(0.00f),
     "gaussian noise level")
    ("vis,v", value<bool>(&visualize)->default_value(true), 
     "enable visualization")
    ("corner,c", value<float>(&corner_size)->default_value(0.02f),
     "corner size")
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

void labelTableObjectIntersection(PointCloud<PointXYZRGBA>::Ptr & pc_table, 
				  PointCloud<PointXYZRGBA>::Ptr & pc_object,
				  const float & radius)
{
  vector<int> indices;
  vector<float> distances;
  KdTreeFLANN<PointXYZRGBA>::Ptr tree(new KdTreeFLANN<PointXYZRGBA>);
  tree->setInputCloud(pc_table);
  for (size_t i = 0; i < pc_object->size(); ++i)
  {
    indices.clear();
    tree->radiusSearch(pc_object->points[i], radius, indices, distances);
    if (indices.size() > 0)
    {
      pc_object->points[i].rgba = LBL_EDGE;
      for (size_t j = 0; j < indices.size(); ++j)
      {
	pc_table->points[indices[j]].rgba = LBL_EDGE;
      }
    }
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud<PointXYZRGBA>::Ptr pc_table (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_mug (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_mug_handle (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_bowl (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_milk (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_bottle (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_bottle_top (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_pan (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_pan_handles (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_apple (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_tea (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_herps (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc_herps_top (new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr pc1 (new PointCloud<PointXYZRGBA>);

  Vec o(0.0f, 0.5f, 1.0f);  // scene offset
  float s = 0.005f;          // step_size


  Vec mug(-0.3f, 0.0f, 0.1f);
  Vec bowl(-0.4f, 0.0f, -0.3f);
  Vec milk(-0.55f, 0.0f, 0.15f);
  Vec bottle(0.05f, 0.0f, 0.3f);
  Vec pan(0.5f, 0.0f, 0.25f);
  Vec apple(0.0f, 0.0f, -0.25f);
  Vec tea(0.3f, 0.0f, -0.25f);
  Vec herps(-0.07f, 0.0f, 0.0f);

  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  if (noise != 0) pg.setGaussianNoise(noise);
  pg.setOutputCloud(pc_table);

  // table:
/*
  pg.generateBox(s, o + Vec(0.75f, 0.0f, -0.5f), 
		 - 1.5f * Vec::UnitX(), Vec(0.0f, 0.05f, 0.0f), 1.0f * Vec::UnitZ(), corner_size);
*/
//  pg.generatePlane(s, o + Vec(0.75f, 0.0f, -0.5f),
//		   -1.5f * Vec::UnitX(), 1.0f * Vec::UnitZ());
//  pg.generateEdge(s, o + Vec(0.75f, 0.0f, -0.5f - corner_size),
//		  -1.5f * Vec::UnitX(), corner_size * Vec::UnitZ());

  // mug:
  pg.setOutputCloud(pc_mug);
  pg.generateCylinder(s, o + mug, -0.095f * Vec::UnitY(), 0.04f);
  pg.setOutputCloud(pc_mug_handle);
  pg.generateHandle(s, o + mug + Vec(-0.04f, -0.0475f, 0.0f), 
		    -Vec::UnitZ(), Vec(0.0f, -0.035f, 0.0f), 0.005f, M_PI);
  labelTableObjectIntersection(pc_mug, pc_mug_handle, corner_size);
//  labelTableObjectIntersection(pc_table, pc_mug, corner_size);

  // bowl:
  pg.setOutputCloud(pc_bowl);
  pg.generateSphere(s, o + bowl + Vec(0.0f, -0.1f, 0.0f), Vec::UnitY(), 0.1f, 0.5 * M_PI);
//  labelTableObjectIntersection(pc_table, pc_bowl, corner_size);

  // milk:
  Vec m_height(0.0f, -0.195f, 0.0f);
  Vec m_length(0.09f * sin(0.25 * M_PI), 0.0f, 0.09f * cos(0.25 * M_PI));
  Vec m_width = m_height.cross(m_length).normalized() * 0.06;
  pg.setOutputCloud(pc_milk);
  pg.generateBox(s, o + milk, m_height, m_length, m_width, corner_size);
//  labelTableObjectIntersection(pc_table, pc_milk, corner_size);

  // bottle:
  pg.setOutputCloud(pc_bottle);
  pg.generateCylinder(s, o + bottle, -0.14f * Vec::UnitY(), 0.045f);
  pg.generateCone(s, o + bottle -0.14f * Vec::UnitY(), -0.12f * Vec::UnitY(), 0.045f, 0.015f);
  pg.generateCylinder(s, o + bottle -0.26f * Vec::UnitY() , -0.02f * Vec::UnitY(), 0.015f);
  pg.setOutputCloud(pc_bottle_top);
  pg.generateCirclePlane(s, o + bottle -0.28f * Vec::UnitY(), -Vec::UnitY(), 0.015f);
  labelTableObjectIntersection(pc_bottle, pc_bottle_top, corner_size);
//  labelTableObjectIntersection(pc_table, pc_bottle, corner_size);

  // pan:
  pg.setOutputCloud(pc_pan);
  pg.generateCylinder(s, o + pan, -0.15 * Vec::UnitY(), 0.15f);
  pg.setOutputCloud(pc_pan_handles);
  pg.generateHandle(s, o + pan + Vec(-0.15f, -0.11f, 0.0f), 
		    -Vec::UnitY(), Vec(0.0f, 0.0f, 0.04f), 0.0075f, M_PI);
  pg.generateHandle(s, o + pan + Vec(0.15f, -0.11f, 0.0f), 
		    Vec::UnitY(), Vec(0.0f, 0.0f, 0.04f), 0.0075f, M_PI);
  labelTableObjectIntersection(pc_pan, pc_pan_handles, corner_size);
//  labelTableObjectIntersection(pc_table, pc_pan, corner_size);

  // apple:
  pg.setOutputCloud(pc_apple);
  pg.generateSphere(s, o + apple + -0.035 * Vec::UnitY(), 0.035f);
//  labelTableObjectIntersection(pc_table, pc_apple, corner_size);

  // tea:
  Vec t_height(0.0f, -0.065f, 0.0f);
  Vec t_length(0.12f * sin(-0.25 * M_PI), 0.0f, 0.12f * cos(-0.25 * M_PI));
  Vec t_width = t_height.cross(t_length).normalized() * 0.045;
  pg.setOutputCloud(pc_tea);
  pg.generateBox(s, o + tea, t_height, t_length, t_width, corner_size);
//  labelTableObjectIntersection(pc_table, pc_tea, corner_size);

  // herps:
  pg.setOutputCloud(pc_herps);
  pg.generateCylinder(s, o + herps, -0.1f * Vec::UnitY(), 0.02f);
  pg.setOutputCloud(pc_herps_top);
  pg.generateCirclePlane(s, o + herps -0.1f * Vec::UnitY(), -Vec::UnitY(), 0.02f);
  labelTableObjectIntersection(pc_herps, pc_herps_top, corner_size);
//  labelTableObjectIntersection(pc_table, pc_herps, corner_size);

//  *pc1 += *pc_table;
  *pc1 += *pc_mug;
  *pc1 += *pc_mug_handle;
  *pc1 += *pc_bowl;
  *pc1 += *pc_milk;
  *pc1 += *pc_bottle;
  *pc1 += *pc_bottle_top;
  *pc1 += *pc_pan;
  *pc1 += *pc_pan_handles;
  *pc1 += *pc_apple;
  *pc1 += *pc_tea;
  *pc1 += *pc_herps;
  *pc1 += *pc_herps_top;

  VoxelGrid<PointXYZRGBA> vox;
  vox.setInputCloud(pc1);
  vox.setLeafSize(0.001f, 0.001f, 0.001f );
  vox.filter(*pc1);

  cout << "Generated new Point Cloud:\n" << *pc1 << endl;
  if (file != "")
  {
    io::savePCDFileASCII(file, *pc1);
    cout << "Saved Point Cloud to: " << file << endl;
  }

  if (visualize)
  {
    visualization::PCLVisualizer v;
    v.setBackgroundColor(255, 255, 255);
    v.addCoordinateSystem(0.1);
    ColorHdlRGBA hdl(pc1);
    v.addPointCloud<PointXYZRGBA>(pc1, hdl, "cloud");

    while(!v.wasStopped())
    {
      v.spinOnce(100);
      usleep(100000);
    }
  }
  else
  {
    cout << "Visualization disabled." << endl;
  }
  return 0;
}
