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

#ifndef RVIZ_BUTTONS_H
#define RVIZ_BUTTONS_H

#include <wx/wx.h>
#include <wx/menu.h>
//#include <wx/panel.h>
#include <wx/dialog.h>
#include <wx/msgdlg.h>
//#include <wx/sizer.h>
#include <ros/ros.h>
#include <string.h>
#include "rviz/display.h"
#include "cob_3d_mapping_demonstrator/rviz_buttons_panel.h"

namespace rviz
{
  class RvizButtons : public Display
{
public:
    /// Constructor
    RvizButtons(const std::string& name, VisualizationManager* manager/*wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi*/);
    ~RvizButtons();

    void onEnable();

    void onDisable();


    void targetFrameChanged()
    {
    }

    void fixedFrameChanged()
    {
    }


protected:
    //! stored window manager interface pointer
    //rviz::WindowManagerInterface * m_wmi;


    /*wxStaticText *m_text_status;
    wxStaticText *m_text_object;
    wxStaticText *m_text_timeout;
    wxStaticText *m_text_dist; // distance to closest pregrasp position*/

    ros::ServiceServer service_start_;
    ros::ServiceServer service_timeout_;

    //wxWindow *parent_;
    RvizButtonsPanel* panel_;
    wxFrame* frame_; // temp




//private:
//    DECLARE_EVENT_TABLE()

};
}

#endif // RVIZ_BUTTONS_H
