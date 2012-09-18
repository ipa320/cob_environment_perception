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
 * Date of creation: 11/2011
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

/*!
 * @brief Defines some commonly used labels for primitive geometrires
 *
 */

#ifndef COB_3D_MAPPING_COMMON_LABEL_DEFINES_H_
#define COB_3D_MAPPING_COMMON_LABEL_DEFINES_H_


// --- define colors ---
#define LBL_NAN      0x999999
#define LBL_BORDER   0xFF00FF // magenta
#define LBL_PLANE    0x00CCFF // cyan
#define LBL_EDGE     0x7F0000 // red
#define LBL_EDGE_CVX 0xFF6600 // orange
#define LBL_COR      0xFFCC00 // dark yellow
#define LBL_COR_CVX  0xFF00FF // magenta
#define LBL_CYL      0x007F00 // green
#define LBL_CYL_CVX  0x00FF66 // light green
#define LBL_SPH      0x00007F // blue
#define LBL_SPH_CVX  0x9900FF // purple
#define LBL_UNDEF    0x999999

// --- define SVM labels ---
#define SVM_PLANE 0
#define SVM_EDGE  1
#define SVM_COR   2
#define SVM_SPH   3
#define SVM_CYL   4

#define SVM_EDGE_CVX 1
#define SVM_SPH_CVX  2
#define SVM_CYL_CVX  3

#define SVM_EDGE_CAV 4
#define SVM_SPH_CAV  5
#define SVM_CYL_CAV  6

#define SVM_COR_CVX  7
#define SVM_COR_CAV  8

// --- define integer labels only for feature evaluation ---
#define EVAL_PLANE 0
#define EVAL_EDGE  1
#define EVAL_SPH   2
#define EVAL_CYL   3
#define EVAL_COR   4
#define EVAL_EDGECORNER 5
#define EVAL_CURVED 6

// --- define integer labels for classification ---
#define I_UNDEF  0
#define I_NAN    1
#define I_BORDER 2
#define I_EDGE   3
#define I_PLANE  4
#define I_CYL    5
#define I_SPHERE 6
#define I_CORNER 7
#define NUM_LABELS 8

#endif
