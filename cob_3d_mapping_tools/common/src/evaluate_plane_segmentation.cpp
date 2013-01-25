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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_tools
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2012
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
 *
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

#include <map>
#include <iostream>
#include <iomanip>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_mapping_tools/io.h"
#include "cob_3d_segmentation/polygon_extraction/polygon_types.h"
#include "cob_3d_segmentation/polygon_extraction/polygon_extraction.h"
#include "cob_3d_segmentation/plane_extraction.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

std::string pc_in, ppm_exp, ppm_pred;

// --------- forward declarations:
class Cluster;
typedef std::map<int,Cluster> ClusterMap;
std::ostream& operator<< (std::ostream&, Cluster&);
std::string colorHumanReadable(int);

void printNoMatch(Cluster&);
void compare(std::map<int,Cluster>&, std::map<int,Cluster>&);
void pca(PointCloud::Ptr, Cluster&);
void fillPolygonPtr(Cluster& c, cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint>& contour);
void createClusters(PointCloud::Ptr, std::map<int,Cluster>&);

// --------- definition:
void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("pc_in", value<std::string>(&pc_in), "pcd")
    ("ppm_exp", value<std::string>(&ppm_exp), "ppm expected")
    ("ppm_pred", value<std::string>(&ppm_pred), "ppm predicted")
    ;

  positional_options_description p_opt;
  p_opt.add("pc_in", 1).add("ppm_exp", 1).add("ppm_pred", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("pc_in") || !vm.count("ppm_exp") || !vm.count("ppm_pred"))
  {
    std::cout <<"Compare plane segments" << std::endl;
    std::cout <<"<point cloud pcd> <ppm expected> <ppm predicted>" << std::endl;
    std::cout << options << std::endl;
    exit(0);
  }
}

class Cluster
{
public:
  Cluster(int id_)
  : id(id_)
  , centroid(Eigen::Vector3f::Zero())
  , comp1(Eigen::Vector3f::Zero())
  , comp2(Eigen::Vector3f::Zero())
  , comp3(Eigen::Vector3f::Zero())
  , values(Eigen::Vector3f::Zero())
  , poly(new cob_3d_mapping::Polygon)
    { }

  inline void addBorder(int x, int y) { borders.push_back(cob_3d_segmentation::PolygonPoint(x,y)); }
  inline void addPoint(int idx) { indices.push_back(idx); }

  friend std::ostream& operator<<( std::ostream& out, Cluster& c);

  int id;
  std::vector<int> indices;
  std::vector<cob_3d_segmentation::PolygonPoint> borders;
  //cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint> poly;
  Eigen::Vector3f centroid;
  Eigen::Vector3f comp1;
  Eigen::Vector3f comp2;
  Eigen::Vector3f comp3;
  Eigen::Vector3f values;
  cob_3d_mapping::Polygon::Ptr poly;
};

std::string colorHumanReadable(int id)
{
  std::stringstream ss;
  ss << "0x" << std::setfill('0') << std::setw(6) << std::right << std::hex << id << std::dec;
  return ss.str();
}

std::ostream& operator<< (std::ostream& out, Cluster& c)
{
  out << colorHumanReadable(c.id) << " {";

  // --- Begin Properties ---
  out << "Size: " << std::setw(6) << std::right << c.indices.size();
  out << " N: " << std::setiosflags(std::ios::fixed) << std::setprecision(6)
      << std::setw(9) << std::right << c.comp3(0) <<","
      << std::setw(9) << std::right << c.comp3(1) <<","
      << std::setw(9) << std::right << c.comp3(2);
  //out<< " Comp: " << c.values(2) << ", " << c.values(1) << ", " << c.values(0) << "}";
  // --- End Properties ---

  out<<"}";
  return out;
}

void printNoMatch(Cluster& c)
{
  std::cout << "No Match: " << c << std::endl;
}

void printForFile(ClusterMap& exp, ClusterMap& pred)
{
  int count_no = 0;

  // format config:
  const int padding = 3;
  std::stringstream ss;
  ss << exp.begin()->second;
  int col = ss.str().length() + padding;
  std::string sep = "|   ";
  std::cout <<"  -  \t"<<"  - \t"<<"labeled\t"<<" - \t"<<" - \t"<<"-\t"<<"  - \t\t"
            <<"plane extraction" << std::endl;
  std::cout <<"color\t"<<"size\t"<<"    n_x\t"<<"n_y\t"<<"n_z\t"<<"d\t"<<"area\t\t"
            <<"    n_x\t"<<"n_y\t"<<"n_z\t"<<"d\t"<<"area\t"<<"A_diff\t"<<"dot"<<std::endl;
  for(ClusterMap::iterator it=exp.begin(); it!= exp.end(); ++it)
  {
    cob_3d_mapping::Polygon::Ptr p1 = it->second.poly;
    float area1 = p1->computeArea3d();
    std::cout << colorHumanReadable(it->second.id) << "\t"
              << it->second.indices.size() << "\t"
              << it->second.comp3[0] << "\t"
              << it->second.comp3[1] << "\t"
              << it->second.comp3[2] << "\t"
              << it->second.centroid.dot(it->second.comp3) << "\t"
              << area1 << "\t\t";
    ClusterMap::iterator match = pred.find(it->first);
    if (match == pred.end())
    {
      std::cout << "no match" << std::endl;
      ++count_no;
      continue;
    }
    float alpha = it->second.comp3.dot(match->second.comp3);
    cob_3d_mapping::Polygon::Ptr p2 = match->second.poly;
    float area2 = p2->computeArea3d();
    float area_diff = 0;
    gpc_polygon gpc_a, gpc_b, gpc_diff;
    p1->getGpcStructure(p1->transform_from_world_to_plane, &gpc_a);
    p2->getGpcStructure(p2->transform_from_world_to_plane, &gpc_b);
    gpc_polygon_clip(GPC_XOR, &gpc_a, &gpc_b, &gpc_diff);

    //cob_3d_mapping::Polygon::Ptr p_diff(new cob_3d_mapping::Polygon);
    if(gpc_diff.num_contours != 0)
    {
      p1->applyGpcStructure(p1->transform_from_world_to_plane, &gpc_diff);
      area_diff = p1->computeArea3d();
    }

    std::cout << match->second.comp3[0] << "\t"
              << match->second.comp3[1] << "\t"
              << match->second.comp3[2] << "\t"
              << match->second.centroid.dot(match->second.comp3) << "\t"
              << area2 << "\t"
              << area_diff << "\t"
              << alpha << std::endl;
  }
}

void compare(ClusterMap& exp, ClusterMap& pred)
{
  int count_no = 0;

  // format config:
  const int padding = 3;
  std::stringstream ss;
  ss << exp.begin()->second;
  int col = ss.str().length() + padding;
  std::string sep = "|   ";

  for(ClusterMap::iterator it=exp.begin(); it!= exp.end(); ++it)
  {
    ClusterMap::iterator match = pred.find(it->first);
    if (match == pred.end())
    {
      std::stringstream ss1;
      ss1 << it->second;
      std::cout << std::setw(col) << std::left << ss1.str() << sep
                << std::setw(round(0.3*col)) << " "
                << std::setw(round(0.7*col)) << std::left << "--- No Match ---" << sep << std::endl;
      ++count_no;
      continue;
    }
    float alpha = it->second.comp3.dot(match->second.comp3);

    cob_3d_mapping::Polygon::Ptr p1 = it->second.poly;
    cob_3d_mapping::Polygon::Ptr p2 = match->second.poly;
    float area1 = p1->computeArea3d();
    float area2 = p2->computeArea3d();
    float area_diff = 0;
    gpc_polygon gpc_a, gpc_b, gpc_diff;
    p1->getGpcStructure(p1->transform_from_world_to_plane, &gpc_a);
    p2->getGpcStructure(p2->transform_from_world_to_plane, &gpc_b);
    gpc_polygon_clip(GPC_XOR, &gpc_a, &gpc_b, &gpc_diff);

    //cob_3d_mapping::Polygon::Ptr p_diff(new cob_3d_mapping::Polygon);
    if(gpc_diff.num_contours != 0)
    {
      p1->applyGpcStructure(p1->transform_from_world_to_plane, &gpc_diff);
      area_diff = p1->computeArea3d();
    }

    std::stringstream ss1, ss2;
    ss1 << it->second;
    ss2 << match->second;
    std::cout << std::setw(col) << std::left << ss1.str() << sep
              << std::setw(col) << std::left << ss2.str() << sep
              << "Dot: " << std::setw(9) << std::left << std::setiosflags(std::ios::fixed) << std::setprecision(6) << alpha
              << " A1: "<< area1 << "  A2: "<< area2 << "  A_Diff: " << area_diff << std::endl;
  }
  count_no = exp.size() - count_no;
  std::cout << "Results:" << std::endl
            << std::setw(30) << std::left << "Matching Accuracy"<<"= ("<<count_no<<"/"<<exp.size()<<") "
            << std::setiosflags(std::ios::fixed)<<std::setprecision(2)<<(float)count_no/(float)exp.size()*100.0f << "%" << std::endl;
}

void pca(PointCloud::Ptr cloud, Cluster& c)
{
  for(std::vector<int>::iterator it=c.indices.begin(); it!=c.indices.end(); ++it)
    c.centroid += (*cloud)[*it].getVector3fMap();
  c.centroid /= c.indices.size();

  Eigen::Matrix3f cov(Eigen::Matrix3f::Zero());
  for(std::vector<int>::iterator it=c.indices.begin(); it!=c.indices.end(); ++it)
  {
    Eigen::Vector3f demean = (*cloud)[*it].getVector3fMap() - c.centroid;
    cov += demean * demean.transpose();
  }

  Eigen::Matrix3f eigenvectors;
  pcl::eigen33(cov, eigenvectors, c.values);
  c.values /= c.indices.size();
  c.comp1 = eigenvectors.col(2);
  c.comp2 = eigenvectors.col(1);
  c.comp3 = eigenvectors.col(0);
}

void fillPolygonPtr(Cluster& c, cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint>& contour, PointCloud::Ptr cloud)
{
  c.poly->centroid << c.centroid, 0;
  c.poly->computeAttributes(c.comp3, c.poly->centroid);
  int max_points = -1, max_idx = -1;
  for (size_t i=0; i<contour.polys_.size(); ++i)
  {
    if ((int)contour.polys_[i].size() > max_points) { max_points = contour.polys_[i].size(); max_idx = i; }
  }

  for (size_t i=0; i<contour.polys_.size(); ++i)
  {
    c.poly->contours.push_back(std::vector<Eigen::Vector3f>());
    if ((int)i == max_idx)
    {
      c.poly->holes.push_back(false);
      std::vector<cob_3d_segmentation::PolygonPoint>::iterator it = contour.polys_[i].begin();
      for( ; it != contour.polys_[i].end(); ++it)
        c.poly->contours.back().push_back( (*cloud)[cob_3d_segmentation::PolygonPoint::getInd(it->x, it->y)].getVector3fMap() );
    }
    else
    {
      c.poly->holes.push_back(true);
      std::vector<cob_3d_segmentation::PolygonPoint>::reverse_iterator it = contour.polys_[i].rbegin();
      for( ; it != contour.polys_[i].rend(); ++it)
        c.poly->contours.back().push_back( (*cloud)[cob_3d_segmentation::PolygonPoint::getInd(it->x, it->y)].getVector3fMap() );
    }
  }
}

void createClusters(PointCloud::Ptr cloud, ClusterMap& cmap)
{
  int w = cloud->width, h = cloud->height, s = cloud->size();
  int mask[] = { -w, 1, w, -1 };
  int mask_size = 4;

  for(int idx=0; idx < s; ++idx)
  {
    int id = (*cloud)[idx].rgba;
    int x = idx % w;
    int y = idx / w;
    int count = 0;

    ClusterMap::iterator it = cmap.find(id);
    if (it == cmap.end())
    {
      //std::cout << "Create new cluster: " << colorHumanReadable(id) << std::endl;
      it = cmap.insert( std::pair<int,Cluster>(id, Cluster(id)) ).first;
    }
    if (y == 0 || y == h-1 || x == 0 || x == w-1)
    {
      it->second.addBorder(x,y);
    }
    else
    {
      for(int i=0;i<mask_size;++i) { if (id != (*cloud)[idx+mask[i]].rgba) { ++count; } }
      if (count < 1) { it->second.addPoint(idx); }
      else { it->second.addBorder(x,y); }
    }
  }

  cob_3d_segmentation::PolygonExtraction pe;
  ClusterMap::iterator it=cmap.begin();
  while(it!=cmap.end())
  {
    if (it->first == 0xFFFFFF || it->first == LBL_UNDEF || it->second.indices.size() <= 5)
    {
      //std::cout << "Delete Cluster " << colorHumanReadable(it->first) << std::endl;
      cmap.erase(it++);
    }
    else
    {
      //std::cout<< it->second << std::endl;
      cob_3d_segmentation::PolygonContours<cob_3d_segmentation::PolygonPoint> poly;
      pe.outline(w, h, it->second.borders, poly);
      pca(cloud, it->second);
      fillPolygonPtr(it->second, poly, cloud);
      ++it;
    }
  }
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);

  PointCloud::Ptr pc_exp(new PointCloud);
  PointCloud::Ptr pc_pred(new PointCloud);
  pcl::PCDReader pcd;
  cob_3d_mapping_tools::PPMReader ppm;
  if (pcd.read(pc_in, *pc_exp) == -1) exit(0);
  *pc_pred = *pc_exp;
  if (ppm.mapRGB(ppm_exp, *pc_exp, false) == -1) { std::cout<<"Mapping error ["<<ppm_exp<<"]"<<std::endl; exit(0); }
  if (ppm.mapRGB(ppm_pred, *pc_pred, false) == -1) { std::cout<<"Mapping error ["<<ppm_pred<<"]"<<std::endl; exit(0); }

  ClusterMap exp;
  createClusters(pc_exp, exp);

  ClusterMap pred;
  createClusters(pc_pred, pred);

  printForFile(exp, pred);

  return 0;
}
