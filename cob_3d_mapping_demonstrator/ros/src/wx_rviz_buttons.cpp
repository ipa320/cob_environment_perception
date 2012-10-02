/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_3d_environment_perception_intern
 * ROS package name: cob_3d_mapping_demonstrator
 * Description: Feature Map for storing and handling geometric features
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 03/2012
 * ToDo:
 *
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

#include "cob_3d_mapping_demonstrator/wx_rviz_buttons.h"
#include "rviz/visualization_manager.h"
#include "rviz/window_manager_interface.h"

using namespace std;


namespace rviz
{

  const int ID_BUTTON_START(101);
  const int ID_BUTTON_STOP(102);
  const int ID_BUTTON_RESET(103);
  const int ID_BUTTON_CLEAR(104);
  const int ID_BUTTON_RECOVER(105);

  RvizButtons::~RvizButtons() {

  }

  /**
 Constructor
   */
  RvizButtons::RvizButtons(const std::string& name, VisualizationManager* manager/*wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi */)
  : Display( name, manager ),
    frame_(0)
  //: wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
  //, m_wmi( wmi )
  {
    // Create controls
    //m_button = new wxButton(this, ID_RESET_BUTTON, wxT("Reset map"));
    wxWindow* parent = 0;

    WindowManagerInterface* wm = vis_manager_->getWindowManager();
    if (wm)
    {
      parent = wm->getParentWindow();
    }
    else
    {
      frame_ = new wxFrame(0, wxID_ANY, wxString::FromAscii(name.c_str()), wxDefaultPosition, wxDefaultSize, wxMINIMIZE_BOX | wxMAXIMIZE_BOX | wxRESIZE_BORDER | wxCAPTION | wxCLIP_CHILDREN);
      parent = frame_;
    }

    panel_ = new RvizButtonsPanel(parent, wxString()/*parent, false, this*/);
    //render_panel_->SetSize(wxSize(640, 480));
    if (wm)
    {
      wm->addPane(name, panel_);
    }

    //parent_ = parent;

    /*button_start_ = new wxButton(this, ID_BUTTON_START, wxT("New"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    button_stop_ = new wxButton(this, ID_BUTTON_STOP, wxT("Plan"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    button_reset_ = new wxButton(this, ID_BUTTON_RESET, wxT("Play"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    button_clear_ = new wxButton(this, ID_BUTTON_CLEAR, wxT("Execute"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    button_recover_ = new wxButton(this, ID_BUTTON_RECOVER, wxT("Reset"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);*/

    /*m_text_status = new wxStaticText(this, -1, wxT("status: waiting"));
    m_text_object = new wxStaticText(this, -1, wxT("object: none"));
    m_text_timeout = new wxStaticText(this, -1, wxT("timeout: none"));
    m_text_dist = new wxStaticText(this, -1, wxT("closest pos.: none"));*/

    /*button_start_->Enable(false);
    button_stop_->Enable(false);
    button_reset_->Enable(false);
    button_clear_->Enable(false);
    button_recover_->Enable(false);*/

    //wxSizer *vsizer = new wxBoxSizer(wxVERTICAL); // top sizer


    /*wxSizer *vsizer_top = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Trajectory planning"));
    wxSizer *hsizer_traj_top = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_traj_mid = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_traj_bot = new wxBoxSizer(wxHORIZONTAL);*/


    //wxSizer *vsizer_mes = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Messages"));

    //wxSizer *hsizer_add = new wxStaticBoxSizer(wxHORIZONTAL,this,wxT("Additional controls"));


    /* Trajectory planning related buttons, on top*/
    /*vsizer->Add(button_start_, ID_BUTTON_START);
    vsizer->Add(button_stop_, ID_BUTTON_STOP);
    vsizer->Add(button_reset_, ID_BUTTON_RESET);
    vsizer->Add(button_clear_, ID_BUTTON_CLEAR);
    vsizer->Add(button_recover_, ID_BUTTON_RECOVER);*/


    /* Status messages*/
    /*vsizer_mes->Add(m_text_status);
    vsizer_mes->Add(m_text_object);
    vsizer_mes->Add(m_text_timeout);
    vsizer_mes->Add(m_text_dist);*/


    //vsizer->SetSizeHints(this);
    ///*this->*/panel_->SetSizerAndFit(vsizer);

    //ros::NodeHandle nh;
    //service_start_ = nh.advertiseService("arm_nav_start",&RvizButtons::nav_start,this);

  }

  void RvizButtons::onEnable()
  {
    if (frame_)
    {
      frame_->Show(true);
    }
    else
    {
      WindowManagerInterface* wm = vis_manager_->getWindowManager();
      wm->showPane(panel_);
    }
  }

  void RvizButtons::onDisable()
  {
    if (frame_)
    {
      frame_->Show(false);
    }
    else
    {
      WindowManagerInterface* wm = vis_manager_->getWindowManager();
      wm->closePane(panel_);
    }
  }

}
///////////////////////////////////////////////////////////////////////////////

