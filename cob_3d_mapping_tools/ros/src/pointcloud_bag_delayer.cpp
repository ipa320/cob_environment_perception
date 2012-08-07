#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_tools/impl/bag_delayer.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_bag_delayer");
  cob_3d_mapping_tools::bag_delayer<sensor_msgs::PointCloud2> delayer;
  if(!delayer.init(argc, argv)) exit(0);
  delayer.run();
  exit(0);
}
