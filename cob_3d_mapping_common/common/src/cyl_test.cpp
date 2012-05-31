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
//
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
//
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
	x_axis1 << 0.352741,-0.934953,-0.037902;
	axes1.push_back(x_axis1);
	y_axis1 << -0.414317,-0.192376,0.889569;
	axes1.push_back(y_axis1);
	c1->id = 0;
	z_axis1 << 0.838997,0.298084,0.455225;
	axes1.push_back(z_axis1);
	c1->axes_=axes1;
	 v1 << -0.079715, -0.025713, 0.952958;
	 contour1.push_back(v1);
	 v1 << 0.082163, -0.016274, 0.773076;
	 contour1.push_back(v1);
	 v1 << -0.082365, 0.015221, 0.342597;
	 contour1.push_back(v1);
	 v1 << -0.071542, -0.043559, 0.878156;
	 contour1.push_back(v1);
	 c1->contours.push_back(contour1);
	 origin1 << 0.000000,0.000000,0.000000;
	 c1->origin_=origin1;
	 c1->r_=0.083759;
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
	x_axis2 << -0.321493,0.946519,-0.027263;
	axes2.push_back(x_axis2);
	y_axis2 << 0.439006,0.123478,-0.889959;
	axes2.push_back(y_axis2);
	c2->id = 0;
	z_axis2 << 0.838997,0.298084,0.455225;
	axes2.push_back(z_axis2);
	c2->axes_=axes2;
	 v2 << 0.027604, 0.079080, 0.227066;
	 contour2.push_back(v2);
	 v2 << -0.015843, 0.082248, 0.472817;
	 contour2.push_back(v2);
	 v2 << 0.063693, -0.054396, 0.015684;
	 contour2.push_back(v2);
	 v2 << 0.008452, -0.083332, 0.004078;
	 contour2.push_back(v2);
	 c2->contours.push_back(contour2);
	 origin2 << 0.000000,0.000000,0.000000;
	 c2->origin_=origin2;
	 c2->r_=0.083759;
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
	x_axis3 << 0.345189,0.355142,-0.868745;
	axes3.push_back(x_axis3);
	y_axis3 << 0.420629,-0.886014,-0.195068;
	axes3.push_back(y_axis3);
	c3->id = 0;
	z_axis3 << 0.838997,0.298084,0.455225;
	axes3.push_back(z_axis3);
	c3->axes_=axes3;
	 v3 << 0.040813, -0.073144, 0.036411;
	 contour3.push_back(v3);
	 v3 << 0.047857, -0.068741, 0.348308;
	 contour3.push_back(v3);
	 v3 << 0.068345, -0.048421, 0.744265;
	 contour3.push_back(v3);
	 v3 << -0.079371, 0.026756, 0.702106;
	 contour3.push_back(v3);
	 c3->contours.push_back(contour3);
	 origin3 << 0.000000,0.000000,0.000000;
	 c3->origin_=origin3;
	 c3->r_=0.083759;
	 c3->holes.push_back(0);
	 c3->debug_=false;
	 c3->merged=1;


	//~~Cylinder 4~
	CylinderPtr  c4  =CylinderPtr(new Cylinder());
	Eigen::Vector3f x_axis4,y_axis4,z_axis4;
	std::vector<Eigen::Vector3f> axes4;
	Eigen::Vector3f origin4;
	 std::vector<Eigen::Vector3f> contour4;
	 Eigen::Vector3f v4;
	x_axis4 << -0.540907,0.547891,0.638150;
	axes4.push_back(x_axis4);
	y_axis4 << 0.059192,0.781640,-0.620914;
	axes4.push_back(y_axis4);
	c4->id = 0;
	z_axis4 << 0.838997,0.298084,0.455225;
	axes4.push_back(z_axis4);
	c4->axes_=axes4;
	 v4 << 0.078715, 0.028628, 0.354389;
	 contour4.push_back(v4);
	 v4 << 0.069083, 0.047363, 0.432644;
	 contour4.push_back(v4);
	 v4 << -0.053271, 0.064636, 0.844118;
	 contour4.push_back(v4);
	 v4 << -0.083059, 0.010811, 0.580308;
	 contour4.push_back(v4);
	 c4->contours.push_back(contour4);
	 origin4 << 0.000000,0.000000,0.000000;
	 c4->origin_=origin4;
	 c4->r_=0.083759;
	 c4->holes.push_back(0);
	 c4->debug_=false;
	 c4->merged=1;







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
//c_map.push_back(c3);
c_map.push_back(c4);


c1->isMergeCandidate(c_map,limits,intersections);
std::vector<CylinderPtr> merge_candidates ;

for(int i=0;i<(int)intersections.size();i++)
{

	  merge_candidates.push_back(c_map[intersections[i]]);
}
std::cout<<"intersections size: ="<<intersections.size()<<std::endl;

std::cout<<"merge_candidates size: ="<<merge_candidates.size()<<std::endl;

c1->merge(merge_candidates);

//
//
Cylinder& result=*merge_candidates[0];
//result.unrolled_.debug_output("result_polygon");
	 std::cout<<"goa-tz MERGED--> new vertices"<<std::endl;
	 for (int i = 0; i < (int) result.contours.size(); ++i) {
		 for (int j = 0; j < (int) result.contours[i].size(); ++j) {

	 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<result.contours[i][j]<<std::endl<<std::endl;
		}
	}





std::cout <<"goa-tz --> done"<< std::endl;

return 1;
}

