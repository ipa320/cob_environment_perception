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
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_tools
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2012
 * ToDo:
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

#include "cob_3d_mapping_tools/test_gui.h"

bool MainApp::OnInit()
{
  gui = Gui::Core::Create();

  f_main = new FrameMain( _("Test Gui"), wxPoint(200,50), wxSize(640, 480), this);
  f_tools = new FrameTools( _("Tools"), wxPoint(10, 50), wxSize(100, 400), this);
  f_main->Show(true);
  f_tools->Show(true);
  SetTopWindow(f_main);
  return true;
}

FrameMain::FrameMain(const wxString& title, const wxPoint& pos, const wxSize& size, MainApp* app)
  : wxFrame(NULL, -1, title, pos, size), app_(app)
{
  //stat_log = new wxTextCtrl(this, -1, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxTE_RICH);
  image_ = new wxImage(640, 480, true);

  CreateStatusBar();
  SetStatusText( _("Main Frame Status") );
}

FrameTools::FrameTools(const wxString& title, const wxPoint& pos, const wxSize& size, MainApp* app)
  : wxMiniFrame(NULL, -1, title, pos, size, wxCAPTION | wxRESIZE_BORDER | wxCLOSE_BOX ), app_(app)
{
  bt_tool_open = new wxButton(this, BT_TOOL_Open, _("Open"), wxPoint(0,0), wxSize(90,50), 0);
}


void FrameTools::OnToolOpen(wxCommandEvent& event)
{
  wxFileDialog* file_dialog = new wxFileDialog(
    this, _("Select a file to open"), wxEmptyString, wxEmptyString,
    //_("Text files (*.txt)|*.txt|C++ Source Files (*.cpp, *.hpp)|*.cpp;*.hpp| C Source files (*.c)|*.c|C header files (*.h)|*.h"),
    _("Images (*.ppm, *.png)|*.ppm;*.png"),
    wxFD_OPEN, wxDefaultPosition);

  if (file_dialog->ShowModal() == wxID_OK)
  {
    app_->f_main->new_file_ = file_dialog->GetPath();
    app_->f_main->image_->LoadFile(app_->f_main->new_file_, wxBITMAP_TYPE_PNG);
    //app_->f_main->SetTitle(wxString(_("Edit - ")) << file_dialog->GetFilename());
  }
  file_dialog->Destroy();
}



BEGIN_EVENT_TABLE(FrameTools, wxFrame)
    EVT_BUTTON(BT_TOOL_Open,  FrameTools::OnToolOpen)
END_EVENT_TABLE()


IMPLEMENT_APP(MainApp)

/*
int main (int argc, char** argv)
{
  typedef Gui::ResourceTypes::OrganizedPointCloud rOpc;
  typedef Gui::ViewTypes::Color vColor;

  Gui::Core* gui = Gui::Core::Create();
  Gui::Resource<std::string>::Ptr r1 = gui->addResource("res1", std::string("a string"));
  Gui::Resource<rOpc>::Ptr r2
    = gui->addResource("res2", rOpc());

  Gui::View<rOpc, vColor>::Ptr v1 = r2->createView<Gui::ViewTypes::Color>("view1_on_r2", 1);
  Gui::View<std::string, vColor>::Ptr v2 = r1->createView<Gui::ViewTypes::Color>("view1_on_r1", 1);
  Gui::View<std::string, Gui::ViewTypes::Depth_Z>::Ptr v3 = r1->createView<Gui::ViewTypes::Depth_Z>("view2_on_r1", 1);

  v1->show();
  v2->show();
  v3->show();

  return 0;
}
*/

