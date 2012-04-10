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

#include "cob_3d_mapping_demonstrator/rviz_buttons_panel.h"
#include <wx/sizer.h>

using namespace std;


namespace rviz
{

const int ID_BUTTON_START(101);
const int ID_BUTTON_STOP(102);
const int ID_BUTTON_RESET(103);
const int ID_BUTTON_CLEAR(104);
const int ID_BUTTON_RECOVER(105);

RvizButtonsPanel::~RvizButtonsPanel() {

}

/**
 Constructor
 */
RvizButtonsPanel::RvizButtonsPanel(wxWindow *parent, const wxString& title/*, rviz::WindowManagerInterface * wmi */)
  : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    //, m_wmi( wmi )
{
    //parent_ = parent;

    button_start_ = new wxButton(this, ID_BUTTON_START, wxT("Start"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    button_stop_ = new wxButton(this, ID_BUTTON_STOP, wxT("Stop"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    button_reset_ = new wxButton(this, ID_BUTTON_RESET, wxT("Reset"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    button_clear_ = new wxButton(this, ID_BUTTON_CLEAR, wxT("Clear"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    button_recover_ = new wxButton(this, ID_BUTTON_RECOVER, wxT("Recover"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    /*m_text_status = new wxStaticText(this, -1, wxT("status: waiting"));
    m_text_object = new wxStaticText(this, -1, wxT("object: none"));
    m_text_timeout = new wxStaticText(this, -1, wxT("timeout: none"));
    m_text_dist = new wxStaticText(this, -1, wxT("closest pos.: none"));*/

    button_start_->Enable(true);
    button_stop_->Enable(true);
    button_reset_->Enable(true);
    button_clear_->Enable(true);
    button_recover_->Enable(true);

    wxSizer *vsizer = new wxBoxSizer(wxVERTICAL); // top sizer


    /*wxSizer *vsizer_top = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Trajectory planning"));
    wxSizer *hsizer_traj_top = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_traj_mid = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_traj_bot = new wxBoxSizer(wxHORIZONTAL);*/


    //wxSizer *vsizer_mes = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Messages"));

    //wxSizer *hsizer_add = new wxStaticBoxSizer(wxHORIZONTAL,this,wxT("Additional controls"));


    /* Trajectory planning related buttons, on top*/
    vsizer->Add(button_start_, ID_BUTTON_START, wxEXPAND);
    vsizer->Add(button_stop_, ID_BUTTON_STOP, wxEXPAND);
    vsizer->Add(button_reset_, ID_BUTTON_RESET, wxEXPAND);
    vsizer->Add(button_clear_, ID_BUTTON_CLEAR, wxEXPAND);
    vsizer->Add(button_recover_, ID_BUTTON_RECOVER, wxEXPAND);


    /* Status messages*/
    /*vsizer_mes->Add(m_text_status);
    vsizer_mes->Add(m_text_object);
    vsizer_mes->Add(m_text_timeout);
    vsizer_mes->Add(m_text_dist);*/


    vsizer->SetSizeHints(this);
    this->SetSizerAndFit(vsizer);

    //ros::NodeHandle nh;
    //service_start_ = nh.advertiseService("arm_nav_start",&RvizButtons::nav_start,this);
    action_client_ = new actionlib::SimpleActionClient<cob_script_server::ScriptAction>("cob_3d_mapping_demonstrator", true);

}
///////////////////////////////////////////////////////////////////////////////

void RvizButtonsPanel::OnStart(wxCommandEvent& event)
{

   ROS_INFO("On start");
   cob_script_server::ScriptGoal goal;
   goal.function_name = "start";
   // Fill in goal here
   action_client_->sendGoal(goal);
   action_client_->waitForResult(ros::Duration(1.0));
   if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     ROS_INFO("Demonstrator was recovered");
   ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());
   /*srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_new",true) && ros::service::call("arm_nav_new",srv) ) {

     if (srv.response.completed) {

         m_button_new->Enable(false);
         m_button_plan->Enable(true);
         m_button_play->Enable(false);
         m_button_reset->Enable(true);
         m_button_success->Enable(false);
         m_button_failed->Enable(true);

         if (goal_pregrasp) {

           m_text_status->SetLabel(wxString::FromAscii("status: Move gripper to desired position"));


         }
         if (goal_away) m_text_status->SetLabel(wxString::FromAscii("status: Move gripper to safe position"));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_new service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }*/


}

void RvizButtonsPanel::OnStop(wxCommandEvent& event)
{

   ROS_INFO("On stop");
   /*cob_script_server::ScriptGoal goal;
   goal.function_name = "stop";
   // Fill in goal here
   action_client_->sendGoal(goal);
   action_client_->waitForResult(ros::Duration(1.0));
   if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     ROS_INFO("Demonstrator was recovered");
   ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());*/
   action_client_->cancelGoal();
   /*srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_plan",true) && ros::service::call("arm_nav_plan",srv) ) {

     if (srv.response.completed) {

       m_button_new->Enable(false);
       m_button_plan->Enable(false);
       m_button_play->Enable(true);
       m_button_execute->Enable(true);
       //m_button_finish->Enable(false);
       m_button_success->Enable(false);
       m_button_failed->Enable(true);

       m_text_status->SetLabel(wxString::FromAscii("status: Trajectory is ready"));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_plan service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

   }*/


}

void RvizButtonsPanel::OnReset(wxCommandEvent& event)
{

   ROS_INFO("On reset");
   cob_script_server::ScriptGoal goal;
   goal.function_name = "reset";
   // Fill in goal here
   action_client_->sendGoal(goal);
   action_client_->waitForResult(ros::Duration(1.0));
   if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     ROS_INFO("Demonstrator was recovered");
   ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());

   /*srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_play",true) && ros::service::call("arm_nav_play",srv) ) {

     if (srv.response.completed) {

       m_button_new->Enable(false);
       m_button_plan->Enable(false);
       m_button_execute->Enable(true);
       //m_button_finish->Enable(false);
       m_button_success->Enable(false);
       m_button_failed->Enable(true);

       m_text_status->SetLabel(wxString::FromAscii("status: Playing trajectory..."));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_play service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }*/

}

void RvizButtonsPanel::OnClear(wxCommandEvent& event)
{
   ROS_INFO("On clear");
   cob_script_server::ScriptGoal goal;
   goal.function_name = "clear";
   // Fill in goal here
   action_client_->sendGoal(goal);
   action_client_->waitForResult(ros::Duration(1.0));
   if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     ROS_INFO("Demonstrator was recovered");
   ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());

   /*srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_execute",true) && ros::service::call("arm_nav_execute",srv) ) {

     if (srv.response.completed) {

       m_button_plan->Enable(false);
       m_button_execute->Enable(false);
       m_button_reset->Enable(false);
       m_button_play->Enable(false);
       m_button_new->Enable(false);
       //m_button_finish->Enable(true);
       m_button_success->Enable(true);
       m_button_failed->Enable(true);

       m_text_status->SetLabel(wxString::FromAscii("status: Executing trajectory..."));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_execute service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

   }*/

}

void RvizButtonsPanel::OnRecover(wxCommandEvent& event)
{
   ROS_INFO("On recover");
   cob_script_server::ScriptGoal goal;
   goal.function_name = "recover";
   // Fill in goal here
   action_client_->sendGoal(goal);
   action_client_->waitForResult(ros::Duration(1.0));
   if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     ROS_INFO("Demonstrator was recovered");
   ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());

   /*srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_reset",true) && ros::service::call("arm_nav_reset",srv) ) {

     if (srv.response.completed) {

       m_button_plan->Enable(false);
       m_button_execute->Enable(false);
       m_button_reset->Enable(false);
       m_button_play->Enable(false);
       m_button_new->Enable(true);
       //m_button_finish->Enable(true);
       m_button_success->Enable(false);
       m_button_failed->Enable(true);

       // TODO: vratit rameno do puvodni polohy!!!!!

       m_text_status->SetLabel(wxString::FromAscii("status: Ok, try it again"));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_reset service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

     m_button_plan->Enable(false);
     m_button_execute->Enable(false);
     m_button_reset->Enable(false);
     m_button_play->Enable(false);
     m_button_new->Enable(true);
     //m_button_finish->Enable(false);
     m_button_success->Enable(false);
     m_button_failed->Enable(true);

   }*/

}



///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
BEGIN_EVENT_TABLE(RvizButtonsPanel, wxPanel)
    EVT_BUTTON(ID_BUTTON_START,  RvizButtonsPanel::OnStart)
    EVT_BUTTON(ID_BUTTON_STOP,  RvizButtonsPanel::OnStop)
    EVT_BUTTON(ID_BUTTON_RESET,  RvizButtonsPanel::OnReset)
    EVT_BUTTON(ID_BUTTON_CLEAR,  RvizButtonsPanel::OnClear)
    EVT_BUTTON(ID_BUTTON_RECOVER,  RvizButtonsPanel::OnRecover)
END_EVENT_TABLE()
}
