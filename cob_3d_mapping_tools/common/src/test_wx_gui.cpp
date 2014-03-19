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

#include <pcl/features/integral_image_normal.h>

#ifdef PCL_MINOR_VERSION
#if PCL_MINOR_VERSION >= 6
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#endif
#endif

#include "cob_3d_mapping_common/label_defines.h"
#include "cob_3d_mapping_common/polygon.h"
#include "cob_3d_segmentation/plane_extraction.h"
#include "cob_3d_segmentation/polygon_extraction/polygon_types.h"
#include "cob_3d_segmentation/polygon_extraction/polygon_extraction.h"
#include "cob_3d_mapping_tools/io.h"
#include "cob_3d_mapping_tools/gui/impl/core.hpp"



#include <pcl/filters/voxel_grid.h>
#include <pcl/common/eigen.h>
#include <boost/bind.hpp>
#include <iomanip>
#include <set>

typedef pcl::PointXYZRGB PT;
typedef pcl::PointCloud<PT> PointCloud;

class Cluster;
typedef std::map<int,Cluster> ClusterMap;

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
    p1->getGpcStructure(&gpc_a);
    p2->getGpcStructure(&gpc_b);
    gpc_polygon_clip(GPC_XOR, &gpc_a, &gpc_b, &gpc_diff);

    //cob_3d_mapping::Polygon::Ptr p_diff(new cob_3d_mapping::Polygon);
    if(gpc_diff.num_contours != 0)
    {
      p1->applyGpcStructure(&gpc_diff);
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
    p1->getGpcStructure(&gpc_a);
    p2->getGpcStructure(&gpc_b);
    gpc_polygon_clip(GPC_XOR, &gpc_a, &gpc_b, &gpc_diff);

    //cob_3d_mapping::Polygon::Ptr p_diff(new cob_3d_mapping::Polygon);
    if(gpc_diff.num_contours != 0)
    {
      p1->applyGpcStructure(&gpc_diff);
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
      //std::cout << colorHumanReadable(id) << ": ";
      for(int i=0;i<mask_size;++i)
      {
        if (id != (*cloud)[idx+mask[i]].rgba) { ++count; }
        //std::cout << colorHumanReadable((*cloud)[idx+mask[i]].rgba) << ", ";
      }
      //std::cout << std::endl;
      if (count < 1) { it->second.addPoint(idx); }
      else { it->second.addBorder(x,y); }
    }
  }

  cob_3d_segmentation::PolygonExtraction pe;
  ClusterMap::iterator it=cmap.begin();
  while(it!=cmap.end())
  {
    std::cout << it->second.indices.size() << std::endl;
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

void createClustersUsingPlaneExtraction(PointCloud::Ptr cloud, ClusterMap& cmap)
{
  PointCloud::Ptr pc_copy(new PointCloud);
  *pc_copy = *cloud;
  pcl::VoxelGrid<PT> voxel;
  voxel.setSaveLeafLayout(true);
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.03,0.03,0.03);
  voxel.filter(*pc_copy);
  for(PointCloud::iterator it = pc_copy->begin(); it != pc_copy->end(); ++it) { it->rgba = LBL_UNDEF; }

  std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > v_hull_pc;
  std::vector<std::vector<pcl::Vertices> > v_hull_poly;
  std::vector<pcl::ModelCoefficients> v_coef;
  PlaneExtraction pe;
  pe.setSaveToFile(false);
  pe.setClusterTolerance(0.06);
  pe.setMinPlaneSize(150);
  pe.setAlpha(0.2);
  pe.extractPlanes(pc_copy, v_hull_pc, v_hull_poly, v_coef);

  std::vector<int> temp_ids;
  const float rand_max_inv = 1.0f/ RAND_MAX;
  std::cout << "Sizes: idx="<<pe.extracted_planes_indices_.size()
            << " coef="<<v_coef.size()<<std::endl;
  for (int i=0;i<pe.extracted_planes_indices_.size();++i)
  {
    int r = (float)rand() * rand_max_inv * 255;
    int g = (float)rand() * rand_max_inv * 255;
    int b = (float)rand() * rand_max_inv * 255;
    int color = ( r << 16 | g << 8 | b );
    for(std::vector<int>::iterator idx=pe.extracted_planes_indices_[i].begin();idx!=pe.extracted_planes_indices_[i].end();++idx)
    {
      (*pc_copy)[*idx].rgba = color;
    }
    temp_ids.push_back(color);
  }
  for(pcl::PointCloud<PT>::iterator it = cloud->begin(); it!=cloud->end(); ++it)
  {
    if (it->z != it->z) { it->rgba = LBL_UNDEF; }
    else { it->rgba = (*pc_copy)[voxel.getCentroidIndex(*it)].rgba; }
  }

  for(int i=0;i<v_hull_pc.size();++i)
  {
    ClusterMap::iterator it = cmap.insert( std::pair<int,Cluster>(temp_ids[i], Cluster(temp_ids[i])) ).first;
    it->second.comp3 = Eigen::Vector3f(v_coef[i].values[0], v_coef[i].values[1], v_coef[i].values[2]);
    cob_3d_mapping::Polygon& p = *it->second.poly;
    for(int c=0; c<3; c++) p.normal_[c] = v_coef[i].values[c];
    p.d_ = v_coef[i].values[3];
    it->second.centroid = it->second.comp3 * p.d_;
    std::vector<Eigen::Vector3f> pts;
    for(int j=0; j<v_hull_pc[i].size(); j++)
      pts.push_back(v_hull_pc[i].points[j].getVector3fMap());

    p.contours_.push_back(pts);
    p.holes_.push_back(false);
    p.computeCentroid();
    p.computeAttributes(it->second.comp3, p.centroid);
  }
}


#ifdef PCL_MINOR_VERSION
#if PCL_MINOR_VERSION >= 6
void createClustersUsingMultiPlaneSegmentation(PointCloud::Ptr cloud, ClusterMap& cmap)
{
  pcl::PointCloud<pcl::Normal>::Ptr n(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr l(new pcl::PointCloud<pcl::Label>);
  float* distance_map;

  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(40.0f);
  ne.setInputCloud(cloud);
  ne.compute(*n);
  distance_map = ne.getDistanceMap();

  std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > > regions;
  std::vector<pcl::ModelCoefficients> coef;
  std::vector<pcl::PointIndices> inlier_indices, label_indices, boundary_indices;
  pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> omps;
  pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB, pcl::Normal>::Ptr comparator(new pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB, pcl::Normal>(distance_map));
  omps.setComparator(comparator);
  omps.setInputCloud(cloud);
  omps.setInputNormals(n);
  omps.setMinInliers(100);
  omps.setAngularThreshold(5.0f/180.0f*M_PI);
  omps.setMaximumCurvature(0.1); // default 0.001
  omps.setDistanceThreshold(0.02f);
  omps.segmentAndRefine(regions, coef, inlier_indices, l, label_indices, boundary_indices);

  for(pcl::PointCloud<PT>::iterator it = cloud->begin(); it!=cloud->end(); ++it)
    it->rgba = LBL_UNDEF;

  const float rand_max_inv = 1.0f/ RAND_MAX;
  std::vector<int> temp_ids;
  for (size_t i = 0; i < inlier_indices.size(); ++i)
  {
    int r = (float)rand() * rand_max_inv * 255;
    int g = (float)rand() * rand_max_inv * 255;
    int b = (float)rand() * rand_max_inv * 255;
    int color = (r << 16 | b << 8 | b);
    for (size_t j=0; j<inlier_indices[i].indices.size(); ++j)
    {
      (*cloud)[ inlier_indices[i].indices[j] ].rgba = color;
    }
    temp_ids.push_back(color);
  }

  for(int i=0;i<boundary_indices.size();++i)
  {
    ClusterMap::iterator it = cmap.insert( std::pair<int,Cluster>(temp_ids[i], Cluster(temp_ids[i])) ).first;
    it->second.comp3 = Eigen::Vector3f(coef[i].values[0], coef[i].values[1], coef[i].values[2]);
    cob_3d_mapping::Polygon& p = *it->second.poly;
    for(int c=0; c<3; c++) p.normal[c] = coef[i].values[c];
    p.d = coef[i].values[3];
    it->second.centroid = it->second.comp3 * p.d;
    std::vector<Eigen::Vector3f> pts;
    for(int j=0; j<boundary_indices[i].indices.size(); ++j)
      pts.push_back((*cloud)[boundary_indices[i].indices[j]].getVector3fMap());
    for(int j=0; j<inlier_indices[i].indices.size(); ++j)
      it->second.indices.push_back(inlier_indices[i].indices[j]);

    p.contours.push_back(pts);
    p.holes.push_back(false);
    p.computeCentroid();
    p.computeAttributes(it->second.comp3, p.centroid);
  }
}
#endif
#endif



  /*--------------------------*/
 /*---------- MAIN ----------*/
/*--------------------------*/
class MainApp : public wxApp
{
  typedef Gui::ResourceTypes::Image RImg;
  typedef Gui::ResourceTypes::OrganizedPointCloud<PT> RPC;
  typedef Gui::ViewTypes::Color VCol;


  Gui::Core* c;
  PointCloud::Ptr pc_exp;
  PointCloud::Ptr pc_pred;

  ClusterMap exp;
  ClusterMap pred;
  ClusterMap pred_new;

  ClusterMap::iterator c_it;

  Gui::Resource<RImg>* r_tmp;
  Gui::View<RImg,VCol>* v_tmp;
  std::set<int> used_ids;

public:
  MainApp() : pc_exp(new PointCloud), pc_pred(new PointCloud)
  {
    c = Gui::Core::Get();
  }

  void OnClick(wxMouseEvent& event, Gui::Resource<RPC>* rs)
  {
    wxPoint p = event.GetPosition();
    int rgba = rs->getData()->at(p.x,p.y).rgba;
    if(rgba == LBL_UNDEF)
    {
      std::cout<<"you dismissed this cluster! continue..."<<std::endl;
      ++c_it;
      showNext();
      return;
    }
    if (rgba == 0xFFFFFF)
    {
      std::cout<<"you can't use this one! please try again..."<<std::endl;
      return;
    }
    if (used_ids.find(rgba) != used_ids.end())
    {
      std::cout<<"you already used this one! please try again..."<<std::endl;
      return;
    }

    (pred_new.insert(std::pair<int,Cluster>(c_it->first, pred.find(rgba)->second))).first->second.id = c_it->first;
    used_ids.insert(rgba);
    std::cout<<"You matched: "<<colorHumanReadable(rgba)<<std::endl;
    std::cout<<"Size now: " << pred_new.size() << std::endl;

    ++c_it;
    showNext();
  }

  void showNext()
  {
    if(c_it == exp.end())
    {
      std::cout << "you're done here..." <<std::endl;
      std::cout << "these are your results! Hope u like 'em :)\n"<<std::endl;
      printForFile(exp, pred_new);

      if(this->argc == 4)
      {
        ClusterMap::iterator it = pred_new.begin();
        while(it!=pred_new.end())
        {
          for(int i=0; i<it->second.indices.size(); ++i)
          {
            (*pc_pred)[it->second.indices[i]].rgba = it->second.id;
          }
          ++it;
        }
        std::string file_out(wxString(this->argv[3]).mb_str());
        cob_3d_mapping_tools::PPMWriter wppm;
        wppm.writeRGB(file_out, *pc_pred);
      }

      return;
    }
    Gui::cvImagePtr cvmat = r_tmp->getData();
    cv::Vec3b dark = cv::Vec3b((int)50,(int)50,(int)50), id;
    id[0] =  c_it->first        & 0x0000ff;
    id[1] = (c_it->first >>  8) & 0x0000ff;
    id[2] = (c_it->first >> 16) & 0x0000ff;
    //std::cout << c_it->second.indices.size() << std::endl;
    for (int y = 0; y<cvmat->rows; ++y)
    {
      for (int x = 0; x<cvmat->cols; ++x)
      {
        cv::Vec3b& here = (*cvmat)(y,x);
        //if(here == id) continue;
        here = dark;
      }
    }

    std::vector<int>::iterator it = c_it->second.indices.begin(), it_end = c_it->second.indices.end();
    while(it != it_end)
    {
      int v = int(*it) / int(640);
      int u = int(*it) % int(640);
      //
      cv::Vec3b& here = (*cvmat)(v,u);
      here = id;
      ++it;
    }

    if(c_it == exp.begin()) { v_tmp->show(); return; }
    else v_tmp->onDataChanged();
  }

  bool OnInit()
  {
    if (this->argc < 3) { std::cout << "command path_to_pointcloud.pcd path_to_groundtruth.ppm " << std::endl; exit(0); }
    /*if (this->argc == 4)
    {
      std::string file_pcd(wxString(this->argv[1]).mb_str());
      std::string file_ppm(wxString(this->argv[2]).mb_str());
      std::string file_out(wxString(this->argv[3]).mb_str());
      if (pcl::io::loadPCDFile<PT>(file_pcd, *pc_exp) < 0) exit(0);
      cob_3d_mapping_tools::PPMReader ppm;
      cob_3d_mapping_tools::PPMWriter wppm;
      if (ppm.mapRGB(file_ppm, *pc_exp, true) == -1) { exit(0); }
      *pc_pred = *pc_exp;
      //createClustersUsingPlaneExtraction(pc_pred, pred);
      createClustersUsingMultiPlaneSegmentation(pc_pred, pred);
      wppm.writeRGB(file_out, *pc_pred);
      exit(0);
      }
    */

    std::string file_pcd(wxString(this->argv[1]).mb_str());
    std::string file_ppm(wxString(this->argv[2]).mb_str());
    if (pcl::io::loadPCDFile<PT>(file_pcd, *pc_exp) < 0) exit(0);
    cob_3d_mapping_tools::PPMReader ppm;
    if (ppm.mapRGB(file_ppm, *pc_exp, true) == -1) { exit(0); }
    *pc_pred = *pc_exp;
    std::cout << pc_exp->width << " " << pc_exp->height << std::endl;
    createClusters(pc_exp, exp);

    #ifdef PCL_MINOR_VERSION
      #if PCL_MINOR_VERSION >= 6
      createClustersUsingMultiPlaneSegmentation(pc_pred, pred);
      #else
      createClustersUsingPlaneExtraction(pc_pred, pred);
      #endif
    #else
    createClustersUsingPlaneExtraction(pc_pred, pred);
    #endif
    c_it = exp.begin();

    Gui::Resource<RPC>* r_exp = Gui::Core::rMan()->create<RPC>("res_expected", pc_exp);
    Gui::Resource<RPC>* r_pred = Gui::Core::rMan()->create<RPC>("res_predicted", pc_pred);

    Gui::View<RPC,VCol>* v_exp = r_exp->createView<VCol>("Expectation");
    Gui::View<RPC,Gui::ViewTypes::Depth_Z>* v2_exp = r_exp->createView<Gui::ViewTypes::Depth_Z>("depth_exp");
    Gui::View<RPC,VCol>* v_pred = r_pred->createView<VCol>("Prediction");
    r_tmp = Gui::Core::rMan()->create<RImg>("res_copy", file_ppm);
    v_tmp = r_tmp->createView<VCol>("CurrentCluster");

    boost::function<void (wxMouseEvent&, Gui::Resource<RPC>*)> f = boost::bind(&MainApp::OnClick, this, _1, _2);
    v_pred->registerMouseCallback(f);


    v_exp->show();
    v_pred->show();
    Gui::Core::wMan()->moveWindow(v_pred, 700, 0);
    v2_exp->show();
    showNext();
    //v_tmp->show();

    std::cout << "Done init" << std::endl;
    return true;
  }

  int OnExit()
  {
    std::cout << "I shut myself down!" << std::endl;
    return 0;
  }

};

IMPLEMENT_APP(MainApp)
