/*
 * test_segmentation.cpp
 *
 *  Created on: 18.06.2012
 *      Author: josh
 */


#include <gtest/gtest.h>



int main(int argc, char **argv){
  ros::Time::init();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
