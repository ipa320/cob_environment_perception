/*
 * test_segmentation.cpp
 *
 *  Created on: 18.06.2012
 *      Author: josh
 */


//includes needed for testing
#include <gtest/gtest.h>


//includes needed for segmentation (things to test)
#include <cob_3d_segmentation/polygon_extraction/polygon_extraction.h>


struct testSXY
{
  int x,y;

  inline bool operator<(const testSXY& rhs) const
      {
    if(y==rhs.y)
      return x<rhs.x;
    return y<rhs.y;
      }

  static int getInd(const int x, const int y)
  {
    return x*60+y;
  }
};

struct testPOLYGON
{
  std::vector<std::vector<testSXY> > polys_;

  void addPolygon()
  {
    polys_.push_back(std::vector<testSXY>());
  }

  void removePolygon()
  {
    polys_.erase(polys_.end()-1);
  }

  void addPoint(int x, int y)
  {
    testSXY xy={x,y};
    polys_.back().push_back(xy);
  }
};

TEST(Segmentation, polygon_extraction)
{
  Segmentation::PolygonExtraction pe;

  std::vector<testSXY> outs;
  for(int i=0; i<30; i++)
  {
    testSXY xy;

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

  testPOLYGON poly;
  pe.outline(60,60, outs, poly);

  printf("%d polygons first with %d points\n", (int)poly.polys_.size(), (int)poly.polys_[0].size());
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
