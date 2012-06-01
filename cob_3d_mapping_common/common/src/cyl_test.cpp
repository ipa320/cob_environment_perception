/*
 * cyl_test.cpp
 *
 *  Created on: May 24, 2012
 *      Author: goa-tz
 */


#include "cob_3d_mapping_common/cylinder.h"

using namespace cob_3d_mapping;


int main(int argc, char **argv) {

	std::cout <<"goa-tz --> start"<< std::endl;


////####################################################
////Cylinder #1
//
//CylinderPtr  c1  =CylinderPtr(new Cylinder());
//
//c1->id = 0;
//
//
//
//Eigen::Vector3f x_axis1,y_axis1,z_axis1;
//std::vector<Eigen::Vector3f> axes1;
//Eigen::Vector3f origin1;
// std::vector<Eigen::Vector3f> contour1;
// Eigen::Vector3f v1;
//
//
//x_axis1 << 1,0,0;
//axes1.push_back(x_axis1);
//
//y_axis1 << 0,1,0;
//axes1.push_back(y_axis1);
//
//z_axis1 << 0,0,1;
//axes1.push_back(z_axis1);
//
//c1->axes_=axes1;
//
//
//
// v1 << -1, 0, 1;
// contour1.push_back(v1);
// v1 << 1, 0, 1;
// contour1.push_back(v1);
// v1 << 1 ,0 ,-1;
// contour1.push_back(v1);
// v1 << -1, 0 ,-1;
// contour1.push_back(v1);
// c1->contours.push_back(contour1);
//
//
//c1->merged=1;
// origin1 << 0,0,0;
// c1->origin_=origin1;
//
// c1->r_=1;
//
//
//
// c1->holes.push_back(0);
// c1->debug_=false;
// c1->unroll();
//
// if (c1->debug_== true) {
//
// 	 std::cout<<"goa-tz C1--> new vertices"<<std::endl;
// 	 for (int i = 0; i < (int) c1->unrolled_.contours.size(); ++i) {
// 		 for (int j = 0; j < (int) c1->unrolled_.contours[i].size(); ++j) {
//
// 			 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<c1->unrolled_.contours[i][j]<<std::endl<<std::endl;
// 		}
// 	}
// }
//
//
//
//
//
//
////####################################################
////Cylinder  #2
// CylinderPtr  c2  =CylinderPtr(new Cylinder());
//
//
// c2->id = 0;
//
// Eigen::Vector3f x_axis2,y_axis2,z_axis2;
// std::vector<Eigen::Vector3f> axes2;
// Eigen::Vector3f origin2;
//  std::vector<Eigen::Vector3f> contour2;
//  Eigen::Vector3f v2;
//
//
// x_axis2 << 0,-1,0;
// axes2.push_back(x_axis2);
//
// y_axis2 << 1,0,0;
// axes2.push_back(y_axis2);
//
// z_axis2 << 0,0,1;
// axes2.push_back(z_axis2);
//
// c2->axes_=axes2;
//
//
//
//  v2 << 0, 1, 1;
//  contour2.push_back(v2);
//  v2 << 0, -1, 1;
//  contour2.push_back(v2);
//  v2 << 0 ,-1 ,-1;
//  contour2.push_back(v2);
//  v2 << 0, 1 ,-1;
//  contour2.push_back(v2);
//  c2->contours.push_back(contour2);
//
//
//c2->merged=1;
//  origin2 << 0,0,0;
//  c2->origin_=origin2;
//
//  c2->r_=1;
//
//
//
//  c2->holes.push_back(0);
//  c2->debug_=false;
//  c2->unroll();
//
//  if (c2->debug_== true) {
//
//  	 std::cout<<"goa-tz C2--> new vertices"<<std::endl;
//  	 for (int i = 0; i < (int) c2->unrolled_.contours.size(); ++i) {
//  		 for (int j = 0; j < (int) c2->unrolled_.contours[i].size(); ++j) {
//
//  			 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<c2->unrolled_.contours[i][j]<<std::endl<<std::endl;
//  		}
//  	}
//  }
	//~~Cylinder 1~
	CylinderPtr  c1  =CylinderPtr(new Cylinder());
	Eigen::Vector3f x_axis1,y_axis1,z_axis1;
	std::vector<Eigen::Vector3f> axes1;
	Eigen::Vector3f origin1;
	 std::vector<Eigen::Vector3f> contour1;
	 Eigen::Vector3f v1;
	x_axis1 << -0.998931,0.046236,-0.000000;
	axes1.push_back(x_axis1);
	y_axis1 << -0.046236,-0.998931,0.000000;
	axes1.push_back(y_axis1);
	c1->id = 0;
	z_axis1 << 0.000000,0.000000,1.000000;
	axes1.push_back(z_axis1);
	c1->axes_=axes1;
	 v1 << -0.339341, 0.368494, 0.080342;
	 contour1.push_back(v1);
	 c1->contours.push_back(contour1);
	 origin1 << 0.000000,0.000000,0.000000;
	 c1->origin_=origin1;
	 c1->r_=0.500940;
	 c1->holes.push_back(0);
	 c1->debug_=false;
	 c1->merged=1;


	 v1 << -0.471480, 0.169255, 0.592080;
	 contour1.push_back(v1);
	 c1->contours.push_back(contour1);
	 origin1 << 0.000000,0.000000,0.000000;
	 c1->origin_=origin1;
	 c1->r_=0.500940;
	 c1->holes.push_back(0);
	 c1->debug_=false;
	 c1->merged=1;


	 v1 << 0.012288, -0.500789, 0.623528;
	 contour1.push_back(v1);
	 c1->contours.push_back(contour1);
	 origin1 << 0.000000,0.000000,0.000000;
	 c1->origin_=origin1;
	 c1->r_=0.500940;
	 c1->holes.push_back(0);
	 c1->debug_=false;
	 c1->merged=1;


	//~~Cylinder 2~
	CylinderPtr  c2  =CylinderPtr(new Cylinder());
	Eigen::Vector3f x_axis2,y_axis2,z_axis2;
	std::vector<Eigen::Vector3f> axes2;
	Eigen::Vector3f origin2;
	 std::vector<Eigen::Vector3f> contour2;
	 Eigen::Vector3f v2;
	x_axis2 << -0.436181,0.899859,-0.000000;
	axes2.push_back(x_axis2);
	y_axis2 << -0.899859,-0.436181,0.000000;
	axes2.push_back(y_axis2);
	c2->id = 0;
	z_axis2 << 0.000000,0.000000,1.000000;
	axes2.push_back(z_axis2);
	c2->axes_=axes2;
	 v2 << 0.432329, 0.253046, 0.234369;
	 contour2.push_back(v2);
	 c2->contours.push_back(contour2);
	 origin2 << 0.000000,0.000000,0.000000;
	 c2->origin_=origin2;
	 c2->r_=0.500940;
	 c2->holes.push_back(0);
	 c2->debug_=false;
	 c2->merged=1;


	 v2 << -0.498244, 0.051901, 0.996582;
	 contour2.push_back(v2);
	 c2->contours.push_back(contour2);
	 origin2 << 0.000000,0.000000,0.000000;
	 c2->origin_=origin2;
	 c2->r_=0.500940;
	 c2->holes.push_back(0);
	 c2->debug_=false;
	 c2->merged=1;


	 v2 << -0.282440, 0.413724, 0.321632;
	 contour2.push_back(v2);
	 c2->contours.push_back(contour2);
	 origin2 << 0.000000,0.000000,0.000000;
	 c2->origin_=origin2;
	 c2->r_=0.500940;
	 c2->holes.push_back(0);
	 c2->debug_=false;
	 c2->merged=1;


	//~~Cylinder 3~
	CylinderPtr  c3  =CylinderPtr(new Cylinder());
	Eigen::Vector3f x_axis3,y_axis3,z_axis3;
	std::vector<Eigen::Vector3f> axes3;
	Eigen::Vector3f origin3;
	 std::vector<Eigen::Vector3f> contour3;
	 Eigen::Vector3f v3;
	x_axis3 << 0.695692,-0.718340,-0.000000;
	axes3.push_back(x_axis3);
	y_axis3 << 0.718340,0.695692,-0.000000;
	axes3.push_back(y_axis3);
	c3->id = 0;
	z_axis3 << 0.000000,0.000000,1.000000;
	axes3.push_back(z_axis3);
	c3->axes_=axes3;
	 v3 << 0.421749, -0.270312, 0.281687;
	 contour3.push_back(v3);
	 c3->contours.push_back(contour3);
	 origin3 << 0.000000,0.000000,0.000000;
	 c3->origin_=origin3;
	 c3->r_=0.500940;
	 c3->holes.push_back(0);
	 c3->debug_=false;
	 c3->merged=1;


	 v3 << 0.500630, -0.017605, 0.176316;
	 contour3.push_back(v3);
	 c3->contours.push_back(contour3);
	 origin3 << 0.000000,0.000000,0.000000;
	 c3->origin_=origin3;
	 c3->r_=0.500940;
	 c3->holes.push_back(0);
	 c3->debug_=false;
	 c3->merged=1;


	 v3 << -0.197825, -0.460224, 0.517959;
	 contour3.push_back(v3);
	 c3->contours.push_back(contour3);
	 origin3 << 0.000000,0.000000,0.000000;
	 c3->origin_=origin3;
	 c3->r_=0.500940;
	 c3->holes.push_back(0);
	 c3->debug_=false;
	 c3->merged=1;




//####################################################
//Is merge???

//  std::cout<<"c1"<<c1->unrolled_.normal<<std::endl;
//  std::cout<<"c2"<<c2->unrolled_.normal<<std::endl;

std::vector<CylinderPtr> c_map;
std::vector<int> intersections;
merge_config limits;
limits.angle_thresh=0.97;
limits.d_thresh= 0.1;

//c_map.push_back(c2);
c_map.push_back(c3);
c_map.push_back(c2);


c1->isMergeCandidate(c_map,limits,intersections);
std::vector<CylinderPtr> merge_candidates ;

for(int i=0;i<(int)intersections.size();i++)
{

	  merge_candidates.push_back(c_map[intersections[i]]);
}
std::cout<<"intersections size: ="<<intersections.size()<<std::endl;

std::cout<<"merge_candidates size: ="<<merge_candidates.size()<<std::endl;


if(merge_candidates.size()>0){
c1->merge(merge_candidates);
//
//

Cylinder& result=*merge_candidates[0];
//result.unrolled_.debug_output("result_polygon");
//	 std::cout<<"goa-tz MERGED--> new vertices"<<std::endl;
//	 for (int i = 0; i < (int) result.contours.size(); ++i) {
//		 for (int j = 0; j < (int) result.contours[i].size(); ++j) {
//
//	 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<result.contours[i][j]<<std::endl<<std::endl;
//		}
//	}
//
	 result.unrolled_.debug_output("result");
}
//
//
//
//
//
//std::cout <<"goa-tz --> done"<< std::endl;

return 1;
}

