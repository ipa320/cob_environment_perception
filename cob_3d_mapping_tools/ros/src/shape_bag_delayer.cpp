#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_tools/impl/bag_delayer.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shape_bag_delayer");
  cob_3d_mapping_tools::bag_delayer<cob_3d_mapping_msgs::ShapeArray> delayer;
  if(!delayer.init(argc, argv)) exit(0);
  delayer.run();
}
