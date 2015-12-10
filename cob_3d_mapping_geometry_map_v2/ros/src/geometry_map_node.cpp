#include <mapping_node.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "geometry_map_v2");
  
  GeometryNode node;

  ros::spin();

  return 0;
}
