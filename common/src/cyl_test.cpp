/*
 * cyl_test.cpp
 *
 *  Created on: May 24, 2012
 *      Author: goa-tz
 */


#include "cob_3d_mapping_common/cylinder.h"

using namespace cob_3d_mapping;



void
transform_cylinder(CylinderPtr & c_ptr,Eigen::Affine3f& trafo)
{


	Cylinder & c=*c_ptr;

 	 for (int i = 0; i < (int) c.contours.size(); ++i) {
 		 for (int j = 0; j < (int) c.contours[i].size(); ++j) {


 			 c.contours[i][j]=trafo*c.contours[i][j];


 		}
 	}

c.origin_=trafo*c.origin_;
std::cout<<"origin\n"<<c.origin_<<std::endl;

for (int i = 0; i < 3; ++i) {

	c.axes_[i]=trafo.rotation()*c.axes_[i];
//	std::cout<<"axis -"<<i<<" \n"<<c.axes_[i]<<std::endl;
}


float roll,pitch,yaw,x,y,z;
pcl::getTranslationAndEulerAngles(trafo,x,y,z,roll,pitch,yaw);


//	std::cout<<" x= "<<x<<" y= "<<z<<" z= "<<z<<" roll= "<<roll<<" pitch= "<<pitch<<" yaw= "<<yaw<<std::endl;
}

int main(int argc, char **argv) {

	std::cout <<"goa-tz --> start"<< std::endl;


//####################################################
//Cylinder #1

CylinderPtr  c1  =CylinderPtr(new Cylinder());

c1->id = 0;



Eigen::Vector3f x_axis1,y_axis1,z_axis1;
std::vector<Eigen::Vector3f> axes1;
Eigen::Vector3f origin1;
 std::vector<Eigen::Vector3f> contour1;
 Eigen::Vector3f v1;


x_axis1 << 1,0,0;
axes1.push_back(x_axis1);

y_axis1 << 0,0,1;
axes1.push_back(y_axis1);

z_axis1 << 0,1,0;
axes1.push_back(z_axis1);

c1->axes_=axes1;



 v1 << -1, 0, 1;
 contour1.push_back(v1);
 v1 << 1, 0, 1;
 contour1.push_back(v1);
 v1 << 1 ,0 ,-1;
 contour1.push_back(v1);
// v1 << -1, 0 ,-1;
// contour1.push_back(v1);
 c1->contours.push_back(contour1);


c1->merged=1;
 origin1 << 0,0,0;
 c1->origin_=origin1;

 c1->r_=1;



 c1->holes.push_back(0);
 c1->debug_=false;

 if (c1->debug_== true) {

 	 std::cout<<"goa-tz C1--> new vertices"<<std::endl;
 	 for (int i = 0; i < (int) c1->unrolled_.contours.size(); ++i) {
 		 for (int j = 0; j < (int) c1->unrolled_.contours[i].size(); ++j) {

 			 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<c1->unrolled_.contours[i][j]<<std::endl<<std::endl;
 		}
 	}
 }






//####################################################
//Cylinder  #2
 CylinderPtr  c2  =CylinderPtr(new Cylinder());


 c2->id = 0;

 Eigen::Vector3f x_axis2,y_axis2,z_axis2;
 std::vector<Eigen::Vector3f> axes2;
 Eigen::Vector3f origin2;
  std::vector<Eigen::Vector3f> contour2;
  Eigen::Vector3f v2;


 x_axis2 << 0,-1,0;
 axes2.push_back(x_axis2);

 y_axis2 << 0,0,1;
 axes2.push_back(y_axis2);

 z_axis2 << 1,0,0;
 axes2.push_back(z_axis2);

 c2->axes_=axes2;



  v2 << 0, 1, 1;
  contour2.push_back(v2);
  v2 << 0, -1, 1;
  contour2.push_back(v2);
  v2 << 0 ,-1 ,-1;
  contour2.push_back(v2);
//  v2 << 0, 1 ,-1;
//  contour2.push_back(v2);
  c2->contours.push_back(contour2);


c2->merged=1;
  origin2 << 0,0,0;
  c2->origin_=origin2;

  c2->r_=1;



  c2->holes.push_back(0);
  c2->debug_=false;

  if (c2->debug_== true) {

  	 std::cout<<"goa-tz C2--> new vertices"<<std::endl;
  	 for (int i = 0; i < (int) c2->unrolled_.contours.size(); ++i) {
  		 for (int j = 0; j < (int) c2->unrolled_.contours[i].size(); ++j) {

  			 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<c2->unrolled_.contours[i][j]<<std::endl<<std::endl;
  		}
  	}
  }


//transform cylinders

  float x,y,z,roll,pitch,yaw;

  x=-1;
  y=1;
  z=1;
  roll=0.3;
  pitch=-0.4;
  yaw=1;


  Eigen::Affine3f trafo;
  pcl::getTransformation(x,y,z,roll,pitch,yaw,trafo);

transform_cylinder(c1,trafo);
transform_cylinder(c2,trafo);


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
//c_map.push_back(c1);
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
//	 result.unrolled_.debug_output("result");
}
//
//
//
//
//
//std::cout <<"goa-tz --> done"<< std::endl;

return 1;
}

