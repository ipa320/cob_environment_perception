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

#ifndef RVIZ_BUTTONS_PANEL_H
#define RVIZ_BUTTONS_PANEL_H

#include <wx/panel.h>
#include <wx/button.h>
#include <ros/ros.h>

#include <cob_script_server/ScriptAction.h>
#include <actionlib/client/simple_action_client.h>


namespace rviz
{
class RvizButtonsPanel : public wxPanel
  {
  public:
    RvizButtonsPanel(wxWindow *parent, const wxString& title);
    ~RvizButtonsPanel();

    virtual void OnStart(wxCommandEvent& event);
    virtual void OnStep(wxCommandEvent& event);
    virtual void OnStop(wxCommandEvent& event);
    virtual void OnReset(wxCommandEvent& event);
    virtual void OnClear(wxCommandEvent& event);
    virtual void OnRecover(wxCommandEvent& event);

  protected:
    wxButton * button_start_;
    wxButton * button_step_;
    wxButton * button_stop_;
    wxButton * button_reset_;
    wxButton * button_clear_;
    wxButton * button_recover_;

    actionlib::SimpleActionClient<cob_script_server::ScriptAction>* action_client_;

  private:
    DECLARE_EVENT_TABLE()
  };
}

#endif //RVIZ_BUTTONS_PANEL_H
