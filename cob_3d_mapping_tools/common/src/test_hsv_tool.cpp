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
#include <boost/bind.hpp>
#include "cob_3d_mapping_tools/gui/impl/core.hpp"

class MainApp : public wxApp
{
  typedef Gui::ResourceTypes::Image Img;
  typedef Gui::ViewTypes::Color Col;

  void OnClick(wxMouseEvent& event, Gui::Resource<Img>* res)
  {
    static int click_count = 0;
    if (event.LeftDClick())
    {
      wxPoint p = event.GetPosition();
      cv::Vec3b& rgb = (*res->getData())(p.y,p.x);
      int h,s,v;
      dc.rgb2hsv(rgb(2),rgb(1),rgb(0),h,s,v);
      std::cout <<"RGB:"<< (int)rgb(2)<<","<<(int)rgb(1)<<","<<(int)rgb(0)<<" HSV:"<<(int)h<<","<<(int)s<<","<<(int)v<<std::endl;
      uint8_t r,g,b;
      dc.hsv2rgb(h,s,v,r,g,b);
      std::cout << "NEW:"<< (int)r<<","<<(int)g<<","<<(int)b<<std::endl;
      dc.addColor(rgb(2),rgb(1),rgb(0));
      ++click_count;
    }
    else if (event.RightDClick())
    {
      uint8_t r,g,b;
      dc.getColor(r,g,b);
      std::cout << "From histogram after "<<click_count<<" points: "<< (int)r<<","<<(int)g<<","<<(int)b<<std::endl;
      click_count = 0;
    }
  }

  bool OnInit()
  {
    if (this->argc < 2) { std::cout << "Please provide an image" << std::endl; exit(0); }

    std::string file(wxString(this->argv[1]).mb_str());
    Gui::Resource<Img>* res = Gui::Core::rMan()->create<Img>("res", file);
    Gui::View<Img,Col>* view = res->createView<Col>("Image");

    boost::function<void (wxMouseEvent&, Gui::Resource<Img>*)> f=boost::bind(&MainApp::OnClick, this, _1, _2);
    view->registerMouseCallback(f);
    view->show();
    return true;
  }
  //Gui::Core* c;
  Gui::Tools::DominantColor dc;
};

IMPLEMENT_APP(MainApp)
