/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception
 * ROS package name: cob_3d_mapping_common
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
 * ToDo:
 *e
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "cob_3d_mapping_common/polygon.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transform.h>
#include <pcl/common/centroid.h>
#include <boost/shared_ptr.hpp>

namespace cob_3d_mapping
{


// NON MEMBER FUNCTIONS
void
getPointOnPolygon(const Eigen::Vector3f &normal,double d,Eigen::Vector3f &point)
{



	float value=fabs(normal(0));
	int direction=0;

	if(fabs(normal(1))>value)
	{

		direction=1;
		value=fabs(normal(1));
	}


	if(fabs(normal(2))>value)
	{
		direction=2;
		value=fabs(normal(2));
	}
	point << 0,0,0;
	point(direction)=-d/normal(direction);

}

void
getCoordinateSystemOnPlane(const Eigen::Vector3f &normal,
                                            Eigen::Vector3f &u, Eigen::Vector3f &v)
{
  v = normal.unitOrthogonal ();
  u = normal.cross (v);
}


void
getTransformationFromPlaneToWorld(const Eigen::Vector3f &normal,const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
{
  Eigen::Vector3f u, v;


  getCoordinateSystemOnPlane(normal, u, v);

//  std::cout << "u " << u <<  std::endl << " v " << v << std::endl;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal,  origin, transformation);
  transformation = transformation.inverse();
}




// Member Functions

//Polygon::Polygon()

void Polygon::assignMembers(Eigen::Vector3f &new_normal,double &new_d){
	d=new_d;
	normal=new_normal;
	Eigen::Vector3f pt_on_polygon;
	getPointOnPolygon(normal,d,pt_on_polygon);
	Eigen::Affine3f transform_from_plane_to_world;
	getTransformationFromPlaneToWorld(normal,pt_on_polygon,transform_from_plane_to_world);
	transform_from_world_to_plane=transform_from_plane_to_world.inverse();


}


void Polygon::assignMembers(){

	Eigen::Vector3f pt_on_polygon;
	getPointOnPolygon(normal,d,pt_on_polygon);
	computeCentroid();
	Eigen::Affine3f transform_from_plane_to_world;
	getTransformationFromPlaneToWorld(normal,pt_on_polygon,transform_from_plane_to_world);
	transform_from_world_to_plane=transform_from_plane_to_world.inverse();

}

void
Polygon::computeCentroid()
{
  //find largest non-hole contour
  unsigned int idx = 0;
  for (unsigned int i = 0; i < contours.size (); i++)
  {
    int max_pts = 0;
    if(!holes[i])
    {
      if((int)contours[i].size()>(int)max_pts)
      {
        max_pts = contours[i].size();
        idx = i;
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ> poly_cloud;
  for (unsigned int j = 0; j < contours[idx].size () ; j++)
  {
    pcl::PointXYZ p;
    p.x = contours[idx][j][0];
    p.y = contours[idx][j][1];
    p.z = contours[idx][j][2];
    poly_cloud.push_back(p);
  }
  pcl::compute3DCentroid(poly_cloud,centroid);
}


double
Polygon::computeArea()
{
  double xi, xi_1, yi, yi_1, area=0;

  //area_. (poly_ptr_->contours.size ());
  double sum;
  for (unsigned int i = 0; i < contours.size (); i++)
  {
    if(holes[i]) continue;
    sum = 0;
    //area_[i] = 0;
    for (unsigned int j = 0; j < contours[i].size (); j++)
    {
      xi = contours[i][j][0];
      yi = contours[i][j][1];
      if (j == (contours[i].size ()) - 1)
      {
        xi_1 = contours[i][0][0];
        yi_1 = contours[i][0][1];
      }
      else
      {
        xi_1 = contours[i][j + 1][0];
        yi_1 = contours[i][j + 1][1];
      }
      sum = sum + (xi * yi_1 - xi_1 * yi);
      /*
       std::cout << " ---------------------------------------" << std::endl;
       std::cout << " isSizeOk: xi-->" << xi << std::endl;
       std::cout << " \t: xi_1-->" << xi_1 << std::endl;
       std::cout << " \nisSizeOk: yi-->" << yi << std::endl;
       std::cout << " \t: yi_1-->" << yi_1 << std::endl;
       std::cout << " isSizeOk: sum-->" << sum << std::endl;
       std::cout << " ++++++++++++++++++++++++++++++++++++++++" <<std::endl;
       */

    }
    area += fabs (sum / 2);
    //std::cout << "\n\t*** Area of polygon ( " << i << " ) = " << area_[i] << std::endl;
  }
  return area;
}




void
Polygon::GpcStructureUsingMap(Eigen::Affine3f& external_trafo, gpc_polygon* gpc_p)
{
  //Eigen::Affine3f transformation_from_plane_to_world;
  //getTransformationFromPlaneToWorld(p.normal, p.contours[0][0], transformation_from_plane_to_world);
  //p.transform_from_world_to_plane = transform_from_world_to_plane;//transformation_from_plane_to_world.inverse();
  //printMapEntry(p);


  gpc_p->num_contours = contours.size();
  gpc_p->hole = (int*)malloc(contours.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(contours.size()*sizeof(gpc_vertex_list));
  //std::cout << "num_contours: " << gpc_p->num_contours << std::endl;
  for(size_t j=0; j<contours.size(); j++)
  {
    //std::cout << j << std::endl;
    gpc_p->hole[j] = holes[j];
    gpc_p->contour[j].num_vertices = contours[j].size();
    //std::cout << "num_vertices: " << gpc_p->contour[j].num_vertices << std::endl;
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));
    for(size_t k=0; k<contours[j].size(); k++)
    {
      //std::cout << p.contours[j][k] << std::endl;
      Eigen::Vector3f point_trans = external_trafo*contours[j][k];

//      std::cout<<point_trans<<std::endl;
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);

      //if(point_trans(2)>0.2 || point_trans(2)<-0.2) std::cout << "z: " << point_trans(2) << std::endl;
//     std::cout << k << ":" << gpc_p->contour[j].vertex[k].x << "," << gpc_p->contour[j].vertex[k].y <<std::endl;
    }
  }
}

void
Polygon::GpcStructure( gpc_polygon* gpc_p)
{
  //printMapEntry(p);
  gpc_p->num_contours = contours.size();
  gpc_p->hole = (int*)malloc(contours.size()*sizeof(int));
  gpc_p->contour = (gpc_vertex_list*)malloc(contours.size()*sizeof(gpc_vertex_list));
  //std::cout << "num_contours: " << gpc_p->num_contours << std::endl;
  for(size_t j=0; j<contours.size(); j++)
  {
    //std::cout << j << std::endl;
    gpc_p->contour[j].num_vertices = contours[j].size();
    gpc_p->hole[j] = 0;
    //std::cout << "num_vertices: " << gpc_p->contour[j].num_vertices << std::endl;
    gpc_p->contour[j].vertex = (gpc_vertex*)malloc(gpc_p->contour[j].num_vertices*sizeof(gpc_vertex));
    for(size_t k=0; k<contours[j].size(); k++)
    {
      //std::cout << p.contours[j][k] << std::endl;
      Eigen::Vector3f point_trans = transform_from_world_to_plane*contours[j][k];
      gpc_p->contour[j].vertex[k].x = point_trans(0);
      gpc_p->contour[j].vertex[k].y = point_trans(1);
      // if(fabs(point_trans(2))>0.01) std::cout << "z: " << point_trans(2) << std::endl;
      //std::cout << k << ":" << gpc_p->contour[j].vertex[k].x << "," << gpc_p->contour[j].vertex[k].y <<std::endl;
    }
  }
}



void
Polygon::merge(std::vector<PolygonPtr>& poly_vec)
{

	Eigen::Vector3f average_normal=normal;
	double average_d=d;
	int merge_counter=1;
//	std::cout << "avg normal " << std::endl << average_normal << std::endl ;
//	std::cout << "p merged " << std::endl << poly_vec[0]->merged << std::endl ;
//	std::cout << "p merged *normal  " << std::endl << poly_vec[0]->merged*poly_vec[0]->normal << std::endl ;


//	outputFile <<"merging with maps:" <<std::endl;

	for(int i=0 ; i< (int) poly_vec.size();i++)
	{
		 Polygon& p_map1 =*(poly_vec[i]);
	//	 outputFile << "map: " << intersections[i] <<std::endl;
		 if(normal.dot(p_map1.normal)<0){
	//	if (p.normal.dot(p_map.normal)<-0.95){
			 p_map1.normal=-p_map1.normal;
			 p_map1.d=-p_map1.d;
		 }
//		 outputFile <<"wird dazu addiert:  d: "<< p_map.d  <<std::endl;
//		 outputFile <<"normale" <<std::endl << p_map.normal << std::endl;


	//	 std::cout << " add normal :" << std::endl << p_map.normal << std::endl;
		 average_normal +=  p_map1.merged* p_map1.normal;
		 average_d +=p_map1.merged * p_map1.d;
		 merge_counter += p_map1.merged;

	}
//	std::cout << "avg normal " << std::endl << average_normal << std::endl ;
//	std::cout << "merge counter " << std::endl << merge_counter << std::endl ;

	average_normal=average_normal/merge_counter;
	average_d=average_d/merge_counter;
	average_normal.normalize();
	average_d /= average_normal.norm();

	Polygon p_average;
	p_average.assignMembers(average_normal,average_d);

////	outputFile << "new System" <<std::endl;
////	outputFile <<"normal" <<std::endl << average_normal<<std::endl<<"d: " <<average_d<<std::endl;
//
////	std::cout << "avg normal " << std::endl << average_normal << std::endl ;
////	std::cout << "avg d :"  << average_d << std::endl ;
//
//
//	Eigen::Vector3f ft_pt;
//
//	getPointOnPolygon(average_normal,average_d,ft_pt);
////	outputFile <<"ft_pt: "<< ft_pt <<std::endl;
//
//
//	Eigen::Affine3f transformation_from_plane_to_world;
//	Eigen::Affine3f transformation_from_world_to_plane;
//	getTransformationFromPlaneToWorld(average_normal, ft_pt, transformation_from_plane_to_world);
//	transformation_from_world_to_plane = transformation_from_plane_to_world.inverse();

	gpc_polygon gpc_C;
	gpc_polygon gpc_B;

//	 ROS_INFO_STREAM("merge trafo " << std::endl << transformation_from_world_to_plane.matrix());

//	outputFile << "new Trafo " << std::endl;
//	outputFile << transformation_from_world_to_plane.matrix()<< std::endl;
//	std::cout<<"poly A"<<std::endl;
	this->GpcStructureUsingMap(p_average.transform_from_world_to_plane, &gpc_C);
//	getGpcStructureUsingMap(p, transformation_from_world_to_plane, &gpc_result);

	for(unsigned int i=0 ; i<poly_vec.size();i++)
	{

		Polygon& p_map2 = *(poly_vec[i]);



//		getGpcStructureUsingMap(p_map, transformation_from_world_to_plane, &gpc_p_map);
//		std::cout<<"poly B"<<std::endl;

		p_map2.GpcStructureUsingMap(p_average.transform_from_world_to_plane,&gpc_B);

		gpc_polygon_clip(GPC_UNION, &gpc_B, &gpc_C, &gpc_C);

//		std::cout<<"poly c"<<std::endl;
//		std::cout<<"num contours"<<gpc_C.num_contours<<std::endl;
//		  for(size_t j=0; j<gpc_C.num_contours; j++){
//			  for(size_t k=0; k<gpc_C.contour[j].num_vertices; k++){
//
//	     std::cout << k << ":" << gpc_C.contour[j].vertex[k].x << "," << gpc_C.contour[j].vertex[k].y <<std::endl;
//
//	}
//
//}

		if (i==0)
		{
//			conversion of oject to boost shared pointer
			p_map2.transform_from_world_to_plane=p_average.transform_from_world_to_plane;
			p_map2.d=p_average.d;
			p_map2.normal=p_average.normal;
			if(merge_counter<9)
			p_map2.merged=merge_counter;
			else
				p_map2.merged=9;


		}

	//	printGpcStructure(&gpc_result);
	//	std::cout << "map" << std::endl;
	//	printGpcStructure(&gpc_p_map);


		if(i!=0)
		{
//			remove map entries
		poly_vec.erase(poly_vec.begin()+i);
	}
//	printGpcStructure(&gpc_result);

	Polygon& p_map3 = *(poly_vec[0]);

	p_map3.contours.resize(gpc_C.num_contours);
	p_map3.holes.resize(gpc_C.num_contours);

	for(int j=0; j<gpc_C.num_contours; j++)
	{
	  p_map3.contours[j].resize(gpc_C.contour[j].num_vertices);
	  p_map3.holes[j] = gpc_C.hole[j];
//          std::cout << "contour " << j << " is " << gpc_result.hole[j] << std::endl;
	  for(int k=0; k<gpc_C.contour[j].num_vertices; k++)
	  {
		//TODO: set z to something else?
		Eigen::Vector3f point(gpc_C.contour[j].vertex[k].x, gpc_C.contour[j].vertex[k].y, 0);
		p_map3.contours[j][k] = p_map3.transform_from_world_to_plane.inverse()*point;
//		p_map3.d=average_d;
//		p_map3.normal=average_normal;
		//TODO: update normal, d, transformation...?
	  }
	}

	}
}


void
//Polygon::merge_intersect(std::vector<PolygonPtr> poly_vec,std::vector<int>& intersections,std::vector<double>& limit_vec)
Polygon::isMergeCandidate(std::vector<PolygonPtr>& poly_vec,merge_config& limits,std::vector<int>& intersections)

{


  for(size_t i=0; i< poly_vec.size(); i++)
  {

    Polygon& p_map = *(poly_vec[i]);



    if((fabs(p_map.normal.dot(normal)) > limits.angle_thresh && fabs(p_map.d-d) < limits.d_thresh))

    {


    	bool is_intersected= this->isMergeCandidate_intersect(p_map);

        if(is_intersected == true)
        {
          //std::cout << "no intersection with map " << i << std::endl;
          //std::cout << p.normal << std::endl;
          continue;
        }
        // std::cout << "intersection with map " << i << std::endl;
        intersections.push_back(i);

    }

  }


  }

bool Polygon::isMergeCandidate_intersect(Polygon& p_map){

	bool is_intersected;
    gpc_polygon gpc_result;
    gpc_polygon gpc_p_merge;
    gpc_polygon gpc_p_map;


//      Eigen::Affine3f transformation_from_world_to_plane;

//      transformation_from_world_to_plane = transform_from_world_to_plane;
//      std::cout<<"this:"<<std::endl;
      this->GpcStructureUsingMap(transform_from_world_to_plane, &gpc_p_merge);
//      std::cout<<"map:"<<std::endl;
      p_map.GpcStructureUsingMap(transform_from_world_to_plane,&gpc_p_map);


      gpc_polygon_clip(GPC_INT,&gpc_p_merge,&gpc_p_map,&gpc_result);

     if(gpc_result.num_contours == 0)
     {
			 is_intersected=false;
     }
	else
	{
			is_intersected=true;
	}

    return is_intersected;
}




void Polygon::debug_output(std::string name){

	 std::cout<<"Start Debug Output~~~~~~ "<<name.c_str()<<" ~~~~~~~~~~~~~~~~~~"<<std::endl;

		 for (int i = 0; i < (int) this->contours.size(); ++i) {
			 for (int j = 0; j < (int) this->contours[i].size(); ++j) {

		 std::cout<<"contor "<<i<<" vertex "<<j <<"  : "<<std::endl<<"~~~~"<<std::endl<<this->contours[i][j]<<std::endl<<std::endl;
			}
		}
		 std::cout<<"normal: "<<std::endl;
		 std::cout<< normal<<std::endl;
		 std::cout<<"End Debug Output~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl<<std::endl;
}




}//namespace
