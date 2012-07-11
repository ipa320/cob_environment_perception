/*
 * interactive_markers.h
 *
 *  Created on: 07.06.2012
 *      Author: josh
 */

#ifndef INTERACTIVE_MARKERS_H_
#define INTERACTIVE_MARKERS_H_


#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <cob_3d_mapping_slam/slam/node.h>


namespace Slam
{

  class InteractiveMarkers
  {
    interactive_markers::InteractiveMarkerServer server_;

  public:
    InteractiveMarkers();

    template<typename OBJECT_CONTEXT>
    void addNode(Node<OBJECT_CONTEXT>::Ptr node);
  };
}


#endif /* INTERACTIVE_MARKERS_H_ */
