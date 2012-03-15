/**
 * $Id: init.cpp 327 2012-03-10 12:16:01Z stancl $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#include "rviz/plugin/type_registry.h"

#include "cob_3d_mapping_rviz_plugins/shape_display.h"

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<rviz::ShapeDisplay>("ShapeDisplay");
}


