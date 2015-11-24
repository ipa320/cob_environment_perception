Travis-CI: [![Build Status](https://travis-ci.org/ipa320/cob_environment_perception.svg?branch=indigo_dev)](https://travis-ci.org/ipa320/cob_environment_perception)

PCL 1.7 and electric compatibility:

-> use: svn co http://svn.pointclouds.org/ros/branches/electric/perception_pcl_unstable perception_pcl_unstable
-> rosmake perception_pcl_unstable
-> in perception_pcl_unstable/pcl/build/pcl_trunk/tools/CMakeLists.txt comment:
      PCL_ADD_EXECUTABLE(pcl_tiff2pcd ${SUBSYS_NAME} tiff2pcd.cpp)
      target_link_libraries(pcl_tiff2pcd pcl_common pcl_io)
   to provide support for older boost version
-> in perception_pcl_unstable/pcl/build/pcl_trunk/features/CMakeLists.txt:
      uncomment all lines for RSD ( .../rsd.h, .../rsd.hpp, .../rsd.cpp)

Interface of MLS in PCL 1.7 has changed quite a bit. Therefore, the current programs using MLS
require some fixes to work properly with 1.7.

