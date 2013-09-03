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

#include <QPushButton>
#include <QVBoxLayout>

#include "cob_3d_mapping_demonstrator/rviz_buttons.h"


using namespace std;


namespace cob_environment_perception
{


  /**
 Constructor
   */
  RvizButtons::RvizButtons( QWidget* parent )
  : rviz::Panel( parent )
  {

    QPushButton* start_button = new QPushButton("Start Movement");
    start_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QPushButton* stop_button = new QPushButton("Stop");
    stop_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QPushButton* step_button = new QPushButton("Down");
    step_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QPushButton* reset_button = new QPushButton("Stop Processing");
    reset_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QPushButton* clear_button = new QPushButton("Clear");
    clear_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QPushButton* recover_button = new QPushButton("Start Processing");
    recover_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(recover_button);
    layout->addWidget(reset_button);
    layout->addWidget(start_button);
    layout->addWidget(stop_button);
    layout->addWidget(step_button);
    layout->addWidget(clear_button);
    setLayout( layout );

    connect( start_button, SIGNAL( clicked() ), this, SLOT( onStart() ));
    connect( stop_button, SIGNAL( clicked() ), this, SLOT( onStop() ));
    connect( step_button, SIGNAL( clicked() ), this, SLOT( onStep() ));
    connect( reset_button, SIGNAL( clicked() ), this, SLOT( onReset() ));
    connect( clear_button, SIGNAL( clicked() ), this, SLOT( onClear() ));
    connect( recover_button, SIGNAL( clicked() ), this, SLOT( onRecover() ));

    action_client_ = new actionlib::SimpleActionClient<cob_script_server::ScriptAction>("cob_3d_mapping_demonstrator", true);
  }


  RvizButtons::~RvizButtons() {

  }

  void RvizButtons::onStart()
  {
     ROS_INFO("On start");
     cob_script_server::ScriptGoal goal;
     goal.function_name = "start";
     // Fill in goal here
     action_client_->sendGoal(goal);
     action_client_->waitForResult(ros::Duration(1.0));
     if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       ROS_INFO("Demonstrator was started");
     ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());
  }

  void RvizButtons::onStop()
  {
     ROS_INFO("On stop");
     action_client_->cancelGoal();
  }

  void RvizButtons::onStep()
  {
     ROS_INFO("On step");
     cob_script_server::ScriptGoal goal;
     goal.function_name = "step";
     // Fill in goal here
     action_client_->sendGoal(goal);
     action_client_->waitForResult(ros::Duration(1.0));
     if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       ROS_INFO("Demonstrator step");
     ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());
  }

  void RvizButtons::onReset()
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
  }

  void RvizButtons::onClear()
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
  }

  void RvizButtons::onRecover()
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

  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( cob_3d_mapping_demonstrator, Buttons, cob_environment_perception::RvizButtons, rviz::Panel )

