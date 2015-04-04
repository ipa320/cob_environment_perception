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
#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <Eigen/Geometry>

#include <libpolypartition/polypartition.h>
#include <pcl/PolygonMesh.h>

#include "ShapeSPH/Util/Signature.h"
#include "ShapeSPH/Util/SphereSampler.h"
#include "ShapeSPH/Util/lineqn.h"
#include "ShapeSPH/Util/SphericalPolynomials.h"
#include "cob_3d_features/invariant_surface_feature/triangle.hpp"

using namespace std;

struct colon_is_space : std::ctype<char> {
  colon_is_space() : std::ctype<char>(get_table()) {}
  static mask const* get_table()
  {
    static mask rc[table_size];
    rc[' '] = std::ctype_base::space;
    rc['\n'] = std::ctype_base::space;
    return &rc[0];
  }
};

bool readOFF(const std::string &fn, pcl::PolygonMesh &mesh) {
	std::ifstream in(fn.c_str());
	if(!in) return false;
	
	std::string line;
	std::getline(in, line);
	if(line.find("OFF")!=0) return false;
	
	int no_verts=0, no_faces=0, no_lines=0;
	
	for(; std::getline(in, line); )   //read stream line by line
	{
		std::istringstream ins(line);      //make a stream for the line itself
		ins.imbue(locale(cin.getloc(), new colon_is_space));

		if(ins >> no_verts >> no_faces >> no_lines)
			break;
	}
	
	pcl::PointCloud<pcl::PointXYZ> points;
	for(; points.size()<no_verts && std::getline(in, line); )   //read stream line by line
	{
		std::istringstream ins(line);      //make a stream for the line itself
		ins.imbue(locale(cin.getloc(), new colon_is_space));

		pcl::PointXYZ pt;
		if(ins >> pt.x >> pt.y >> pt.z)
			points.push_back(pt);
	}
	if(points.size()!=no_verts) return false;
	pcl::toPCLPointCloud2(points, mesh.cloud);
	
	for(; mesh.polygons.size()<no_faces && std::getline(in, line); )   //read stream line by line
	{
		std::istringstream ins(line);      //make a stream for the line itself
		ins.imbue(locale(cin.getloc(), new colon_is_space));

		int n=0;
		if(!(ins >> n)) continue;
		
		pcl::Vertices v;
		int ind;
		for(int i=0; i<n; i++)
			if(ins >> ind)
				v.vertices.push_back(ind);
		if(v.vertices.size()!=n) return false;
		mesh.polygons.push_back(v);
	}
	if(mesh.polygons.size()!=no_faces) return false;
	return true;
}

int main(int argc, char **argv)
{
	namespace po = boost::program_options; 
	
	typedef double Real;
	typedef double Scalar;
	typedef Sampler<Real, Scalar> S;
	
	int num_radii, num_angles;
	
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("class", po::value<std::string>(), "class")
		("filename", po::value<std::string>(), "filename")
		("arff", po::value<std::string>(), "arff")
		("num_radii", po::value<int>(&num_radii)->default_value(8), "num_radii")
		("num_angles", po::value<int>(&num_angles)->default_value(32), "num_angles")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);    

	if (vm.count("help") || vm.count("class")<1 || vm.count("filename")<1 || vm.count("arff")<1) {
		cout << desc << "\n";
		return 1;
	}
	
	pcl::PolygonMesh mesh;
	if(!readOFF(vm["filename"].as<std::string>(), mesh)) {
		std::cerr<<"could not read off file "<<vm["filename"].as<std::string>()<<std::endl;
		return -1;
	}

	S gedt((Real)num_radii/2. , num_radii , num_angles);
	std::vector< FourierKeyS2< Real > > sKeys;
	
	S::Samples samples;
	gedt.getSamples(samples);
	
	//compute from mesh
	pcl::PointCloud<pcl::PointXYZ> points;
	pcl::fromPCLPointCloud2(mesh.cloud, points);
	
	//scale
	pcl::PCA<pcl::PointXYZ> pca;
	//pca.compute(points);
	pca.setInputCloud(points.makeShared());
	Eigen::Matrix3f inv; inv.fill(0);
	inv(2,2)=inv(1,1)=inv(0,0)=100/ std::max(std::max(pca.getEigenValues()(0),pca.getEigenValues()(1)),pca.getEigenValues()(2));
	Eigen::Matrix3f T = pca.getEigenVectors()*inv*pca.getEigenVectors().inverse();
	//T=T.Identity();
	
	std::cout<<pca.getEigenValues()<<std::endl;
	//std::cout<<pca.getEigenVectors()<<std::endl;
	//std::cout<<T*pca.getEigenVectors()<<std::endl;
	
	Eigen::Vector3d off = Eigen::Vector3d::Random();
	Eigen::AngleAxisd rotA(0.25*M_PI, Eigen::Vector3d::Random().normalized());
	Eigen::Matrix3d rot = rotA.matrix();
	
	Scalar area=0;
	time_t start = time(0);
	for(size_t i=0; i<mesh.polygons.size(); i++) {
		for(int j=0; j<(int)mesh.polygons[i].vertices.size()-2; j++) {
//std::cout<<(T*(points[mesh.polygons[i].vertices[j  ]].getVector3fMap()-pca.getMean().head<3>())).transpose()<<std::endl;
			cob_3d_features::invariant_surface_feature::SingleTriangle<Scalar, S::Samples, S::Values> tri(
				(T*(points[mesh.polygons[i].vertices[j  ]].getVector3fMap()-pca.getMean().head<3>())).cast<double>(),
				(T*(points[mesh.polygons[i].vertices[j+1]].getVector3fMap()-pca.getMean().head<3>())).cast<double>(),
				(T*(points[mesh.polygons[i].vertices[j+2]].getVector3fMap()-pca.getMean().head<3>())).cast<double>()
			);
			tri.compute(samples);
			gedt += tri;
			area += tri.area();
		}
	}
	std::cout<<"step 2 finished: "<<(time(0)-start)<<"s"<<std::endl;
		
	//compute harmonics
	gedt.sample(sKeys);

	//generate signature/descriptor
	std::ofstream out(vm["arff"].as<std::string>().c_str(), std::ios_base::app);
	out<<vm["filename"].as<std::string>()<<","<<vm["class"].as<std::string>();
	
	area = std::sqrt(area);
	//area = std::sqrt(sKeys[0](0,0).squareNorm());
	const int bw = sKeys[0].bandWidth();
	Signature< Real > sig( (bw) * int( sKeys.size() ) );
	for( int i=0 ; i<sKeys.size() ; i++ )
	{
		for( int b=0 ; b<bw ; b+=2 )
		{
			Real _norm2 = sKeys[i](b,0).squareNorm();
			for( int j=1 ; j<=b ; j++ )
				_norm2 += sKeys[i](b,j).squareNorm()*2;
			sig[i*bw+b] = Real( sqrt(_norm2) );
			//for( int j=0 ; j<=b ; j++ )
			//	std::cout<<sqrt(sKeys[i](b,j).squareNorm())/area<<" ";
			//std::cout<<sig[i*bw+b]/area<<" ";
			//std::cout<<std::endl;
			
			out<<","<<sig[i*bw+b]/area;
			std::cout<<sig[i*bw+b]/area<<" ";
		}
		std::cout<<std::endl;
	}
	out<<std::endl;
	out.close();
	
	const bool debug=false;
	if(debug) {
		sig.write( "/tmp/sig1" /*, Binary.set*/ );
		for(size_t i=0; i<sKeys.size(); i++)
		{
			char fn[512];
			sprintf(fn, "/tmp/key1_%zu", i);
			sKeys[i].write(fn);
		}
	}	
}
