/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_environment_perception
* \note
* ROS package name: cob_3d_mapping_common
*
* \author
* Author: Thomas Zwölfer, email:georg.arbeiter@ipa.fhg.de
* \author
* Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
*
* \date Date of creation: 05/2012
*
* \brief
* ?
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/


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
std::cout<<"transformed origin\n"<<c.origin_<<std::endl;


	c.sym_axis=trafo.rotation()*c.sym_axis;
	c.normal = trafo.rotation()*c.normal;
//	std::cout<<"axis -"<<i<<" \n"<<c.axes_[i]<<std::endl;
c.normal=trafo.rotation()*c.normal;

float roll,pitch,yaw,x,y,z;
pcl::getTranslationAndEulerAngles(trafo,x,y,z,roll,pitch,yaw);
//	std::cout<<" x= "<<x<<" y= "<<z<<" z= "<<z<<" roll= "<<roll<<" pitch= "<<pitch<<" yaw= "<<yaw<<std::endl;

c.computeAttributes(c.sym_axis, c.normal, c.origin_);	//	configure unrolled polygon
}


int main(int argc, char **argv) {


//####################################################
//Cylinder #1

CylinderPtr  c1  =CylinderPtr(new Cylinder());
c1->id = 0;

Eigen::Vector3f x_axis1,y_axis1,z_axis1;
std::vector<Eigen::Vector3f> axes1;
Eigen::Vector3f origin1;
 std::vector<Eigen::Vector3f> contour1;
 std::vector<std::vector<Eigen::Vector3f> > contours1;

 Eigen::Vector3f v1;


 x_axis1 << 1,0,0;
 		axes1.push_back(x_axis1);

 		y_axis1 << 0,0,1;
 		axes1.push_back(y_axis1);

 		z_axis1 << 0,1,0;
 		axes1.push_back(z_axis1);

 		c1->sym_axis = y_axis1;
 		c1->normal = z_axis1;
 		c1->r_=1;



 		v1 << -1, 0, 1;
 		contour1.push_back(v1);
 		v1 << 0, 1, 1;
 		contour1.push_back(v1);
 		v1 << 1, 0, 1;
 		contour1.push_back(v1);
 		v1 << 1 ,0 ,-1;
 		contour1.push_back(v1);
 		v1 << 0, 1, -1;
 		contour1.push_back(v1);
 		v1 << -1, 0 ,-1;
 		contour1.push_back(v1);

 		c1->merged=0;
 		origin1 << 0,0,0;
 		c1->origin_=origin1;

 		c1->holes.push_back(0);
 		c1->debug_=true;

 		contours1.push_back(contour1);
 		c1->ContoursFromList(contours1);






// c1->r_=1;




// if (c1->debug_== true) {
//
// 	 std::cout<<"goa-tz C1--> new vertices"<<std::endl;
// 	 for (int i = 0; i < (int) c1->contours.size(); ++i) {
// 		 for (int j = 0; j < (int) c1->contours[i].size(); ++j) {
//
// 			 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<c1->contours[i][j]<<std::endl<<std::endl;
// 		}
// 	}
// }






//####################################################
//Cylinder  #2
 CylinderPtr  c2  =CylinderPtr(new Cylinder());
 c2->id = 0;

 Eigen::Vector3f x_axis2,y_axis2,z_axis2;
 std::vector<Eigen::Vector3f> axes2;
 Eigen::Vector3f origin2;
  std::vector<Eigen::Vector3f> contour2;
  std::vector<std::vector<Eigen::Vector3f> > contours2;
  Eigen::Vector3f v2;


  x_axis2 << 1,0,0;
  		axes2.push_back(x_axis2);

  		y_axis2 << 0,0,1;
  		axes2.push_back(y_axis2);

  		z_axis2 << 0,1,0;
  		axes2.push_back(z_axis2);

c2->sym_axis=y_axis2;
c2->normal = z_axis2;
c2->r_=1;



  		v2 << -1, 0, 1;
  		contour2.push_back(v2);
  		v2 << 0.1, 0, 1;
  		contour2.push_back(v2);
  		v2 << 1, 0, 1;
  		contour2.push_back(v2);
  		v2 << 1 ,0 ,-1;
  		contour2.push_back(v2);
  		v2 << 0.1, 0, -1;
  		contour2.push_back(v2);
  		v2 << -1, 0 ,-1;
  		contour2.push_back(v2);

  		c2->merged=0;
  		origin1 << 0,0,0;
  		c2->origin_=origin2;

  		c2->holes.push_back(0);
  		c2->debug_=false;

  		contours2.push_back(contour2);
  		c2->ContoursFromList(contours2);



//  if (c2->debug_== true) {
//
//  	 std::cout<<"goa-tz C2--> new vertices"<<std::endl;
//  	 for (int i = 0; i < (int) c2->contours.size(); ++i) {
//  		 for (int j = 0; j < (int) c2->contours[i].size(); ++j) {
//
//  			 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<c2->contours[i][j]<<std::endl<<std::endl;
//  		}
//  	}
//  }


//transform cylinders

  float x,y,z,roll,pitch,yaw;

  x=1;
  y=2;
  z=3;
  roll=0.2;
  pitch=0.4;
  yaw=1;


  Eigen::Affine3f trafo;
  pcl::getTransformation(x,y,z,roll,pitch,yaw,trafo);

transform_cylinder(c1,trafo);
transform_cylinder(c2,trafo);




std::string s_c1 = "c1->unrolled";

std::string s_c2 = "c2->unrolled";
//#######################################################
//  completion test
//  c1->completeCylinder();
//  c2->completeCylinder();
//  std::cout<<"X-axis = \n"<<c1->axes_[0]<<std::endl;
//  std::cout<<"X-axis = \n"<<c2->axes_[0]<<std::endl;


  std::cout<<"r = "<<c1->r_<<std::endl;
  std::cout<<"r = "<<c2->r_<<std::endl;




//#######################################################
//	cylinders from octave script
//#######################################################




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

c_map.push_back(c2);
//c_map.push_back(c3);


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




std::vector<std::vector<Eigen::Vector3f> > con3d;

c1->getCyl3D(con3d);

std::cout<<"c1 - size"<<c1->contours[0].size()<<std::endl;
std::cout<<"con3d - size"<<con3d[0].size()<<std::endl;


for (int i = 0; i < con3d.size(); ++i) {
	for (int j = 0; j < con3d[i].size(); ++j) {

		std::cout<<con3d[i][j]<<std::endl<<std::endl;

	}


}
}





return 1;
}

