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
//#include <boost/bind.hpp>

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_features/organized_normal_estimation_omp.h"
#include "cob_3d_mapping_tools/gui/impl/core.hpp"

class MainApp : public wxApp
{
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::Normal NormalT;
  typedef Gui::ResourceTypes::OrganizedPointCloud<NormalT> Pc;
  typedef Gui::ViewTypes::Normal<0> Nx;
  typedef Gui::ViewTypes::Normal<1> Ny;
  typedef Gui::ViewTypes::Normal<2> Nz;

  void OnClick(wxMouseEvent& event, Gui::Resource<Pc>* res)
  {
    static int click_count = 0;
    if (event.LeftDClick())
    {
    }
  }

  bool OnInit()
  {
    if (this->argc < 2) { std::cout << "Please provide an image" << std::endl; exit(0); }

    std::string file(wxString(this->argv[1]).mb_str());

    p.reset(new pcl::PointCloud<PointT>);
    n.reset(new pcl::PointCloud<NormalT>);
    l.reset(new pcl::PointCloud<PointLabel>);

    PCDReader r;
    r.read(file,*p);
    cob_3d_features::OrganizedNormalEstimationOMP<PointT, NormalT, PointLabel> one;
    one.setInputCloud(p);
    one.setOutputLabels(l);
    one.setPixelSearchRadius(8,2,2); //radius,pixel,circle
    one.setSkipDistantPointThreshold(6.0);
    one.compute(*n);
    Gui::Resource<Pc>* res = Gui::Core::rMan()->create<Pc>("res", n);
    Gui::View<Pc,Nx>* viewx = res->createView<Nx>("Normals x");
    Gui::View<Pc,Ny>* viewy = res->createView<Ny>("Normals y");
    Gui::View<Pc,Nz>* viewz = res->createView<Nz>("Normals z");

    //boost::function<void (wxMouseEvent&, Gui::Resource<Img>*)> f=boost::bind(&MainApp::OnClick, this, _1, _2);
    //view->registerMouseCallback(f);
    viewx->show();
    viewy->show();
    viewz->show();
    return true;
  }

  pcl::PointCloud<PointT>::Ptr p;
  pcl::PointCloud<NormalT>::Ptr n;
  pcl::PointCloud<PointLabel>::Ptr l;
};

IMPLEMENT_APP(MainApp)
