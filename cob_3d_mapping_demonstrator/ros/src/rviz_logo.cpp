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
 * Date of creation: 04/2012
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

#include <QLabel>
#include <QPixmap>
#include <QHBoxLayout>
#include <QDesktopWidget>
#include <QRect>
#include <QSize>
#include <QApplication>

#include <rviz/visualization_manager.h>
#include <ros/package.h>

#include "cob_3d_mapping_demonstrator/rviz_logo.h"

using namespace std;


namespace cob_environment_perception
{

  /**
 Constructor
   */
  RvizTitle::RvizTitle( QWidget* parent )
  :   rviz::Panel( parent )
      , manager_( NULL )

  {
    string path = ros::package::getPath("cob_3d_mapping_demonstrator") + "/ros/files/logo_title.jpg";

    QPixmap pixmap(path.c_str());
    //resize image if it is larger than screen size.
    //QDesktopWidget* desktopWidget = QApplication::desktop();
    QRect rect(0,0,1500,100);// = desktopWidget->availableGeometry();

    QSize size(rect.width() , rect.height());
    //resize as per your requirement..

    image_ = new QPixmap(pixmap.scaledToWidth(1500));
    image_label_ = new QLabel();

    image_label_->setScaledContents ( true );
    image_label_->setPixmap(*image_);

    QHBoxLayout* main_layout = new QHBoxLayout;
    main_layout->addWidget(image_label_);
    setLayout(main_layout);


    /*wxWindow* parent = 0;

    WindowManagerInterface* wm = vis_manager_->getWindowManager();
    if (wm)
    {
      parent = wm->getParentWindow();
    }
    else
    {
      frame_ = new wxFrame(0, wxID_ANY, wxString::FromAscii(""), wxDefaultPosition, wxDefaultSize, wxMINIMIZE_BOX | wxMAXIMIZE_BOX | wxRESIZE_BORDER | wxCAPTION | wxCLIP_CHILDREN);
      parent = frame_;
    }

    panel_ = new wxImagePanel(parent, wxString::FromAscii(path.c_str()),wxBITMAP_TYPE_JPEG);


    if (wm)
    {
      wm->addPane(name, panel_);
    }*/
  }

  // Save all configuration data from this panel to the given Config
  // object.  It is important here that you append the key_prefix to all
  // keys you use, so that your settings don't collide with settings
  // from other components.
  /*void RvizTitle::saveToConfig( const std::string& key_prefix, const boost::shared_ptr<rviz::Config>& config )
  {
    //config->set( key_prefix + "/Topic", output_topic_ );
  }

  // Load all configuration data for this panel from the given Config
  // object.  It is important here that you append the key_prefix to all
  // keys you use, so that your settings don't collide with settings
  // from other components.
  void RvizTitle::loadFromConfig( const std::string& key_prefix, const boost::shared_ptr<rviz::Config>& config )
  {
    /*std::string topic;
    config->get( key_prefix + "/Topic", &topic );
    output_topic_editor_->setText( QString::fromStdString( topic ));
    updateTopic();*/
  //}


  void RvizTitle::configChanged()
  {
    cout << "bla";
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( cob_3d_mapping_demonstrator, Title, cob_environment_perception::RvizTitle, rviz::Panel )

///////////////////////////////////////////////////////////////////////////////

