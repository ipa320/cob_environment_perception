/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
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

#include <fstream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Image.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <iostream>
#include <Eigen/Geometry>

/*
#include "ShapeSPH/Util/SphereSampler.h"
#include "ShapeSPH/Util/Signature.h"
#include "ShapeSPH/Util/lineqn.h"
#include "ShapeSPH/Util/SphericalPolynomials.h"
#include "cob_3d_features/invariant_surface_feature/triangle.hpp"
*/


// Packages Includes:
#include "cob_3d_features/impl/invariant_surface_feature.hpp"
//#include "cob_3d_features/impl/invariant_surface_feature_unit_tests.hpp"
#include "cob_3d_features/impl/invariant_surface_feature_debug.hpp"
#include <cob_3d_segmentation/quad_regression/quad_regression.h>
#include "cob_3d_segmentation/quad_regression/polygon.h"
 

template<class Samples, class Values>
void simple_test(Eigen::Vector3d n, const Samples &samples, Values &vals) {
	vals.resize(samples.size());
double s=0;
	for(size_t i=0; i<samples.size(); i++) {
		for(size_t j=0; j<samples[i].size(); j++) {
//			vals[i].push_back( (i==0&&j==3)?1:0 );
//			vals[i].push_back(samples[i][j].dot(n)>0.4);
			vals[i].push_back(std::complex<double>(samples[i][j].dot(n), 0));
//std::cout<<samples[i][j].transpose()<<" = ";
//std::cout<<vals[i].back()<<" ";
s+=std::abs(vals[i].back());
//if(j%8==7)
//		std::cout<<std::endl;
		}
//		std::cout<<std::endl;
	}
std::cout<<"sum "<<s<<std::endl;
}

int main(int argc, char **argv)
{

	std::string fn = "/home/josh/Downloads/MaleLow.obj";
	//fn = "/home/josh/Downloads/batmobile.obj";
	fn = "/home/josh/Downloads/bunny.ply";
	//fn = "/home/josh/Downloads/Beautiful Girl.obj";
	//fn = "/home/josh/Downloads/Sovereign 5.obj";
	fn = "/home/josh/Downloads/cylinder.obj";
	fn = "/home/josh/Downloads/sphere.obj";
	//fn = "/home/josh/Downloads/prowler_c.obj";
	//fn = "/home/josh/Downloads/skinner.obj";
	std::string fn2 = "/home/josh/Downloads/Beautiful Girl.obj";
	//fn2 = "/home/josh/Downloads/Sovereign 5.obj";
	fn2 = "/home/josh/Downloads/sphere.obj";
	//fn2 = "/home/josh/Downloads/skinner.obj";
	//fn2 = "/home/josh/Downloads/cylinder.obj";

	typedef float Real;
	typedef double Scalar;
	typedef Sampler<Real, Scalar> S;

	srand(time(NULL));

	const int num_radii = 8;
	const int num_angles = 32;
	S gedt((Real)num_radii , num_radii , num_angles);
	S gedt2((Real)num_radii , num_radii , num_angles);
	std::vector< FourierKeyS2< Real > > sKeys;
	S::Samples samples;
	gedt.getSamples(samples);

	Eigen::Vector3d p1 = Eigen::Vector3d::Random(), p2 = Eigen::Vector3d::Random(), p3 = Eigen::Vector3d::Random(), off = Eigen::Vector3d::Random(), n1=Eigen::Vector3d::Random(),n2, off2;
	Eigen::Vector3d p4 = Eigen::Vector3d::Random(), p5 = Eigen::Vector3d::Random(), p6 = Eigen::Vector3d::Random();
	
	off2(0)=off2(1)=off2(2)=0;

//p1(0)=0;
//p1(1)=0;
//p1(2)=0;

//p2(0)=1;
//p2(1)=0;
//p2(2)=1;

//p3(0)=1;
//p3(1)=1;
//p3(2)=1;

	Eigen::AngleAxisd rotA(0.33*M_PI, Eigen::Vector3d::Random().normalized());
	Eigen::Matrix3d rot = rotA.matrix();
//	Eigen::AngleAxisd rot(0.5*M_PI, Eigen::Vector3d::UnitX());
	n2 = rot*n1;

	if(1) {		
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri(p1, p2, p3);
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri_opp(-p1, -p2, -p3);
		
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri3(p4, p5, p6);
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri5(p4*2, p5, p6+p2);

		//cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri2(p1+off, p2+off, p3+off);
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri2(rot*p1+off, rot*p2+off, rot*p3+off);
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri4(rot*p4+off, rot*p5+off, rot*p6+off);
		cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri6(rot*(2*p4)+off, rot*p5+off, rot*(p6+p2)+off);

		std::cout<<"nd1 "<<n1.dot(p1)<<std::endl;
		std::cout<<"nd2 "<<n2.dot(rot*p1)<<std::endl;

		std::cout<<"area "<<(p2-p1).cross(p3-p1).norm()/2<<std::endl;
		std::cout<<"0 val. "<<std::abs(tri.kernel_lin_tri(Eigen::Vector3d::Zero())+tri3.kernel_lin_tri(Eigen::Vector3d::Zero())+tri5.kernel_lin_tri(Eigen::Vector3d::Zero()))<<std::endl;
		std::cout<<"0 val. "<<std::abs(tri2.kernel_lin_tri(Eigen::Vector3d::Zero())+tri4.kernel_lin_tri(Eigen::Vector3d::Zero())+tri6.kernel_lin_tri(Eigen::Vector3d::Zero()))<<std::endl;

		std::cout<<"single val. "<<std::abs(tri.kernel_lin_tri(n1)+tri3.kernel_lin_tri(n1)+tri5.kernel_lin_tri(n1))<<std::endl;
		//std::cout<<"single val. "<<tri.kernel_lin_tri(n2)<<std::endl;
		//std::cout<<"single val. "<<tri2.kernel_lin_tri(n1)<<std::endl;
		std::cout<<"single val. "<<std::abs(tri2.kernel_lin_tri(n2)+tri4.kernel_lin_tri(n2)+tri6.kernel_lin_tri(n2))<<std::endl;

		std::cout<<"single val. "<<std::abs( tri.kernel_lin_tri(n1,true))<<std::endl;
		std::cout<<"single val. "<<std::abs( tri_opp.kernel_lin_tri(n1,true))<<std::endl;
		std::cout<<"single val. "<<std::abs(tri2.kernel_lin_tri(n2,true))<<std::endl;
		//std::cout<<"single val. "<<std::abs(tri2.kernel_lin_tri(n1,true))<<std::endl;
		
		std::cout<<"sum val. "<<std::abs( tri.kernel_lin_tri(n1)+tri_opp.kernel_lin_tri(n1))<<std::endl;
		
		std::cout<<"t "<<std::polar<float>(1,0)<<std::endl;
		std::cout<<"t "<<std::polar<float>(1,PI)<<std::endl;
		std::cout<<"t "<<std::polar<float>(1,0)+std::polar<float>(1,PI)<<std::endl;

		tri.compute(samples);
		tri2.compute(samples);
		exit(0);
	}

	/*std::cout<<"off: "<<off.transpose()<<std::endl;
	for(size_t i=0; i<((S::Values)tri).size(); i++) {
		for(size_t j=0; j<((S::Values)tri)[i].size(); j++) {
			std::complex<Scalar> a = ((S::Values)tri)[i][j], b = ((S::Values)tri2)[i][j];
			if(std::abs(std::abs(a)-std::abs(b))>0.01) {
				std::cout<<std::abs(a)<<"/"<<std::abs(b)<<" "<<std::endl;
				std::cout<<(a)<<"/"<<(b)<<" "<<std::endl;
				//assert(0);
			}
		}
	}*/


	const bool test=false;
	const int M=atoi(argv[2]), Start=atoi(argv[1]);
	if(!test) {
		FILE *fp = fopen("/tmp/cmp_prec", "wb");
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFile(fn, mesh);
		std::cout<<"step 1 finished"<<std::endl;

		pcl::PointCloud<pcl::PointXYZ> points;
		pcl::fromROSMsg(mesh.cloud, points);
		time_t start = time(0);
		for(size_t i=Start; i<std::min(mesh.polygons.size(), (size_t)M); i++) {
			for(int j=0; j<(int)mesh.polygons[i].vertices.size()-2; j++) {
/*std::cout
					<<(points[mesh.polygons[i].vertices[j  ]].getVector3fMap().cast<double>()+off2).transpose()<<std::endl
					<<(points[mesh.polygons[i].vertices[j+1]].getVector3fMap().cast<double>()+off2).transpose()<<std::endl
					<<(points[mesh.polygons[i].vertices[j+2]].getVector3fMap().cast<double>()+off2).transpose()<<std::endl
<<std::endl;
std::cout
					<<(rot*points[mesh.polygons[i].vertices[j  ]].getVector3fMap().cast<double>()+off+off2).transpose()<<std::endl
					<<(rot*points[mesh.polygons[i].vertices[j+1]].getVector3fMap().cast<double>()+off+off2).transpose()<<std::endl
					<<(rot*points[mesh.polygons[i].vertices[j+2]].getVector3fMap().cast<double>()+off+off2).transpose()<<std::endl
<<std::endl;*/


				cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri(
					points[mesh.polygons[i].vertices[j  ]].getVector3fMap().cast<double>()+off2,
					points[mesh.polygons[i].vertices[j+1]].getVector3fMap().cast<double>()+off2,
					points[mesh.polygons[i].vertices[j+2]].getVector3fMap().cast<double>()+off2
				);
				tri.compute(samples);
				gedt += tri;

//std::cout<<"single val. "<<( tri.kernel_lin_tri(samples[7][42]))<<" "<<tri.area()<<std::endl;
//std::cout<<"at 42 "<<gedt.complex_vals_[7][42]<<std::endl;

				/*std::cout<<"single val. "<<std::abs( tri.kernel_lin_tri(n1))<<" "<<tri.area()<<std::endl;
for(size_t l=0; l<((S::Values)tri)[7].size(); l++) {
	std::cout<<std::abs(((S::Values)tri)[7][l])<<" ";
}
std::cout<<std::endl;*/
			}
		}
for(size_t l=0; l<gedt.complex_vals_[7].size(); l++) {
	double d;
	d=std::abs(gedt.complex_vals_[7][l]);
	fwrite(&d, sizeof(double), 1, fp);
}
fclose(fp);
		std::cout<<"step 2 finished: "<<(time(0)-start)<<"s"<<std::endl;
	}

	if(test) {S::Values vals;
	simple_test(n1, samples, vals);
	gedt+=vals;}
	gedt.sample(sKeys);

	const int bw = sKeys[0].bandWidth();
	Signature< Real > sig( (bw) * int( sKeys.size() ) );
	for( int i=0 ; i<sKeys.size() ; i++ )
	{
		std::cout<<"Key:"<<std::endl;
		for( int b=0 ; b<bw ; b++ )
		{
			Real _norm2 = sKeys[i](b,0).squareNorm();
			//std::cout<<sKeys[i](b,0).squareNorm()<<" ";
			for( int j=1 ; j<=b ; j++ ) {
				_norm2 += sKeys[i](b,j).squareNorm()*2;
				//std::cout<<sKeys[i](b,j).squareNorm()<<" ";
			}
			sig[i*bw+b] = Real( sqrt(_norm2) );
			if(b%2==0)
				std::cout<<sig[i*bw+b]<</*"/"<<sig[i*bw+b]/sig[i*bw]<<*/" ";
		}
		std::cout<<std::endl;
	}
	sig.write( "/tmp/sig1" /*, Binary.set*/ );

	for(size_t i=0; i<sKeys.size(); i++)
	{
		char fn[512];
		sprintf(fn, "/tmp/key1_%d", i);
		sKeys[i].write(fn);
	}


	if(!test) {
		FILE *fp = fopen("/tmp/cmp_prec2", "wb");
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFile(fn2, mesh);
		std::cout<<"step 1 finished"<<std::endl;

		pcl::PointCloud<pcl::PointXYZ> points;
		pcl::fromROSMsg(mesh.cloud, points);
		time_t start = time(0);
		for(size_t i=Start; i<std::min(mesh.polygons.size(), (size_t)M); i++) {
			for(int j=0; j<(int)mesh.polygons[i].vertices.size()-2; j++) {
				cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri(
					rot*points[mesh.polygons[i].vertices[j  ]].getVector3fMap().cast<double>()+off+off2,
					rot*points[mesh.polygons[i].vertices[j+1]].getVector3fMap().cast<double>()+off+off2,
					rot*points[mesh.polygons[i].vertices[j+2]].getVector3fMap().cast<double>()+off+off2
				);
				tri.compute(samples);
				gedt2 += tri;

//std::cout<<"single val. "<<( tri.kernel_lin_tri(samples[7][42]))<<" "<<tri.area()<<std::endl;
//std::cout<<"at 42 "<<gedt2.complex_vals_[7][42]<<std::endl;

				/*std::cout<<"single val. "<<std::abs( tri.kernel_lin_tri(n2))<<" "<<tri.area()<<std::endl;
		std::cout<<"step 2 finished: "<<(time(0)-start)<<"s"<<std::endl;
for(size_t l=0; l<((S::Values)tri)[7].size(); l++) std::cout<<std::abs( ((S::Values)tri)[7][l] )<<" ";
std::cout<<std::endl;*/

				/*cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri2(
					points[mesh.polygons[i].vertices[j  ]].getVector3fMap().cast<double>(),
					points[mesh.polygons[i].vertices[j+1]].getVector3fMap().cast<double>(),
					points[mesh.polygons[i].vertices[j+2]].getVector3fMap().cast<double>()
				);
				tri2.compute(samples);
				for(size_t k=0; k<((S::Values)tri).size(); k++) for(size_t l=0; l<((S::Values)tri)[k].size(); l++)
					if(std::abs( std::abs(((S::Values)tri)[k][l])-std::abs(((S::Values)tri2)[k][l]) )>0.0001) {
						std::cout<<((S::Values)tri)[k][l]-((S::Values)tri2)[k][l]<<std::endl;
						std::cout<<std::abs( std::abs(((S::Values)tri)[k][l])-std::abs(((S::Values)tri2)[k][l]) )<<std::endl;
						tri.kernel_lin_tri(samples[k][l],true);
						tri2.kernel_lin_tri(samples[k][l],true);
						assert(0);
					}*/
			}
		}
for(size_t l=0; l<gedt2.complex_vals_[7].size(); l++) {
	double d;
	d=std::abs(gedt2.complex_vals_[7][l]);
	fwrite(&d, sizeof(double), 1, fp);
}
fclose(fp);
	}

	if(test) {
		FILE *fp1 = fopen("/tmp/cmp_prec", "rb");
		FILE *fp2 = fopen("/tmp/cmp_prec2", "rb");
		double m=0;
		int i=0;
		while(!feof(fp1) && !feof(fp2)) {
			double d1,d2;
			fread(&d1, sizeof(double), 1, fp1);
			fread(&d2, sizeof(double), 1, fp2);
			std::cout<<i<<":\t"<<d1-d2<<" "<<d1<<" "<<d2<<" \t    "<<samples[7][i++].transpose()<<"\n";
			m = std::max(std::abs(d1-d2), m);
		}
		std::cout<<std::endl<<"MAX "<<m<<std::endl;
		fclose(fp1);
		fclose(fp2);
	}


	if(test) {S::Values vals;
	simple_test(n2, samples, vals);
	gedt2+=vals;}
	gedt2.sample(sKeys);

	/*for(int n=0; n<gedt.vals_.size(); n++) {
		std::cout<<"DELTA"<<n<<"\n";
		for(int i=0; i<gedt.vals_[n].size(); i++)
			std::cout<<(gedt.vals_[n][i]-gedt2.vals_[n][i])<<" ";
		std::cout<<std::endl;
		std::cout<<"DELTA"<<n<<"\n";
		for(int i=0; i<gedt.vals_[n].size(); i++)
			std::cout<<(gedt.vals_[n][i])<<" "<<(gedt2.vals_[n][i])<<" "<<std::abs(gedt.complex_vals_[n][i])<<" "<<std::abs(gedt2.complex_vals_[n][i])<<" | ";
			//std::cout<<(gedt.complex_vals_[n][i])<<" "<<(gedt2.complex_vals_[n][i])<<std::abs(gedt.complex_vals_[n][i])<<" "<<std::abs(gedt2.complex_vals_[n][i])<<" | ";
		std::cout<<std::endl;
	}*/

	{
	Signature< Real > sig( (bw) * int( sKeys.size() ) );
	for( int i=0 ; i<sKeys.size() ; i++ )
	{
		std::cout<<"Key:"<<std::endl;
		for( int b=0 ; b<bw ; b++ )
		{
			Real _norm2 = sKeys[i](b,0).squareNorm();
			//std::cout<<sKeys[i](b,0).squareNorm()<<" ";
			for( int j=1 ; j<=b ; j++ ) {
				_norm2 += sKeys[i](b,j).squareNorm()*2;
				//std::cout<<sKeys[i](b,j).squareNorm()<<" ";
			}
			sig[i*bw+b] = Real( sqrt(_norm2) );
			if(b%2==0) std::cout<<sig[i*bw+b]<</*"/"<<sig[i*bw+b]/sig[i*bw]<<*/" ";
		}
		std::cout<<std::endl;
	}
	sig.write( "/tmp/sig2" /*, Binary.set*/ );}

	for(size_t i=0; i<sKeys.size(); i++)
	{
		char fn[512];
		sprintf(fn, "/tmp/key2_%d", i);
		sKeys[i].write(fn);
	}
	
	{
		enum {DEGREE=2};
		typedef cob_3d_features::InvariantSurfaceFeature<Segmentation::S_POLYGON<DEGREE> > ISF;
		ISF isf(16,16);
		
		//parameters
		isf.addRadius(0.25);
		
		//input
		std::ifstream fis("/tmp/blub");
		Segmentation::Segmentation_QuadRegression<pcl::PointXYZ, pcl::PointXYZRGB, Segmentation::QPPF::QuadRegression<DEGREE, pcl::PointXYZ, Segmentation::QPPF::CameraModel_Kinect<pcl::PointXYZ> > > seg;
		seg.serialize(fis);
		
		ISF::PTSurfaceList input(new ISF::TSurfaceList);
		*input = seg.getPolygons();
		std::cout<<"read data"<<std::endl;
		
		//compute
		isf.setInput(input);
		std::cout<<"setInput"<<std::endl;
		isf.compute();
		std::cout<<"compute"<<std::endl;
		ISF::PResultConst oldR = isf.getResult();
		std::cout<<"getResult"<<std::endl;
		
		pcl::io::savePLYFile("map1.ply", 		*isf.dbg_Mesh_of_Map());
	}
}

#if 0
#include <fstream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Image.h>
#include <pcl/io/ply_io.h>

// Packages Includes:
#include "cob_3d_features/impl/invariant_surface_feature.hpp"
#include "cob_3d_features/impl/invariant_surface_feature_unit_tests.hpp"
#include "cob_3d_features/impl/invariant_surface_feature_debug.hpp"
#include <cob_3d_segmentation/quad_regression/quad_regression.h>
#include "cob_3d_segmentation/quad_regression/polygon.h"


TEST(InvariantSurfaceFeature, sampleSurfaces) {
	typedef cob_3d_features::InvariantSurfaceFeature<4,4, Segmentation::S_POLYGON<2>, double, Eigen::Affine3d> ISF;

	//read data
	std::ifstream fis("/tmp/blub");
	Segmentation::Segmentation_QuadRegression<pcl::PointXYZ, pcl::PointXYZRGB, Segmentation::QPPF::QuadRegression<2, pcl::PointXYZ, Segmentation::QPPF::CameraModel_Kinect<pcl::PointXYZ> > > seg;
	seg.serialize(fis);
	ISF::PTSurfaceList input(new ISF::TSurfaceList);
	*input = seg.getPolygons();

	//read data
	std::ifstream fis2("/tmp/blub2");
	seg.serialize(fis2);
	ISF::PTSurfaceList input2(new ISF::TSurfaceList);
	*input2 = seg.getPolygons();

	ISF isf;

	//parameters
	//isf.addRadius(0.2);
	isf.addRadius(0.5);
	//isf.addRadius(1);

	//compute
	isf.setInput(input);
	isf.compute();
	ISF::PResultVectorListConst oldR = isf.getResult();
	pcl::io::savePLYFile("map1.ply", 		*isf.dbg_Mesh_of_Map());

	//compute
	isf.setInput(input2);
	isf.compute();
	ISF::PResultVectorListConst newR = isf.getResult();
	pcl::io::savePLYFile("map2.ply", 		*isf.dbg_Mesh_of_Map());

	//analyse: distinctiveness within same data
	Eigen::MatrixXd cor(oldR->size(),oldR->size());
	for(size_t i=0; i<oldR->size(); i++) {
		for(size_t k=0; k<oldR->size(); k++) {

			cor(i,k)=0;
			for(size_t j=0; j<(*oldR)[i].ft.size(); j++)
				cor(i,k) += (*oldR)[i].ft[j]-(*oldR)[k].ft[j];

		}
	}

	std::cout<<"Correlation\n"<<cor<<std::endl;


	//analyse: distinctiveness within same data
	Eigen::MatrixXd cor2(oldR->size(),newR->size());
	for(size_t i=0; i<oldR->size(); i++) {
		for(size_t k=0; k<newR->size(); k++) {

			cor2(i,k)=0;
			for(size_t j=0; j<(*oldR)[i].ft.size(); j++)
				cor2(i,k) += (*oldR)[i].ft[j]-(*newR)[k].ft[j];

		}
	}

	std::cout<<"Correlation\n"<<cor2<<std::endl;
}

/*TEST(InvariantSurfaceFeature, singleTriangle) {
	typedef cob_3d_features::InvariantSurfaceFeature<4,4, Segmentation::S_POLYGON<1>, double, Eigen::Affine3d> ISF;
	ISF isf;

	//parameters
	isf.addRadius(1);

	//tests
	//EXPECT_TRUE( isf.test_singleTriangle(100) );
	//return;
	pcl::io::savePLYFile("random_submap.ply", 	*isf.test_subsampling_of_Map(5, 0.5));
	pcl::io::savePLYFile("random_map.ply", 		*isf.dbg_Mesh_of_Map());

	isf.compute();
	ISF::PResultVectorListConst oldR = isf.getResult();

	//test translational invariance (for ease only in z direction for now)
	double off = 1.781;
	std::cout<<"moved in z-direction by "<<off<<std::endl;
	//isf.test_addOffset(off*3, off*2, off);
	isf.test_rotate(0.123+M_PI/4);
	isf.compute();
	ISF::PResultVectorListConst newR = isf.getResult();

	pcl::io::savePLYFile("random_map_rotated.ply", 		*isf.dbg_Mesh_of_Map());

	assert(oldR->size()==newR->size());
	for(size_t i=0; i<oldR->size(); i++) {
		assert((*oldR)[i].ft.size()==(*newR)[i].ft.size());

		for(size_t j=0; j<(*oldR)[i].ft.size(); j++) {
			for(size_t k=0; k<ISF::NUM_RADIUS; k++) {
				//std::cout<<i<<" "<<j<<" "<<k<<" "<<oldR->size()<<" "<<newR->size()<<std::endl;
				std::cout<<"dist "<<(*oldR)[i].ft[j]-(*newR)[i].ft[j]<<std::endl;
				std::cout<<( (*oldR)[i].ft[j].vals[k])<<std::endl;
				std::cout<<( (*newR)[i].ft[j].vals[k] )<<std::endl;
				std::cout<<( (*oldR)[i].ft[j].vals[k]-         (*newR)[i].ft[j].vals[k] )<<std::endl<<std::endl;
				EXPECT_TRUE( (*oldR)[i].ft[j].vals[k].isApprox((*newR)[i].ft[j].vals[k], 0.1) );
			}
		}
	}

	Eigen::MatrixXd cor(oldR->size(),oldR->size());
	for(size_t i=0; i<oldR->size(); i++) {
		for(size_t k=0; k<newR->size(); k++) {

			cor(i,k)=0;
			for(size_t j=0; j<(*oldR)[i].ft.size(); j++)
				cor(i,k) += (*oldR)[i].ft[j]-(*newR)[k].ft[j];

		}
	}

	std::cout<<"Correlation\n"<<cor<<std::endl;

}
*/

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#endif
