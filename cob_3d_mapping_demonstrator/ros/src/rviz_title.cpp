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

#include <ros/package.h>

#include "cob_3d_mapping_demonstrator/rviz_title.h"

using namespace std;


namespace cob_environment_perception
{

  /**
 Constructor
   */
  RvizTitle::RvizTitle( QWidget* parent )
  :   rviz::Panel( parent )
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
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( cob_3d_mapping_demonstrator, Title, cob_environment_perception::RvizTitle, rviz::Panel )

