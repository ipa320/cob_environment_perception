/*
 * test_segmentation.cpp
 *
 *  Created on: 18.06.2012
 *      Author: josh
 */


//includes needed for testing
#include <gtest/gtest.h>


//includes needed for segmentation (things to test)
#include <cob_3d_segmentation/polygon_extraction/polygon_types.h>
#include <cob_3d_segmentation/polygon_extraction/polygon_extraction.h>



TEST(cob_3d_segmentation, polygon_extraction)
{
  cob_3d_segmentation::PolygonExtraction pe;

  std::vector<cob_3d_segmentation::PolygonPoint> outs;
  for(int i=0; i<30; i++)
  {
    cob_3d_segmentation::PolygonPoint xy;

    xy.x=i+10;
    xy.y=10;
    outs.push_back(xy);

    xy.x=11+i;
    xy.y=40;
    outs.push_back(xy);

    xy.x=10;
    xy.y=11+i;
    outs.push_back(xy);

    xy.x=40;
    xy.y=10+i;
    outs.push_back(xy);
  }

  cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint> poly;
  pe.outline(60,60, outs, poly);

  printf("%d polygons first with %d points\n", (int)poly.polys_.size(), (int)poly.polys_[0].size());
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
