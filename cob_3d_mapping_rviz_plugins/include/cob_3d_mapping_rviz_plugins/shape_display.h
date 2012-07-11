/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SHAPE_DISPLAY_H
#define SHAPE_DISPLAY_H

#include "rviz/display.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/forwards.h"

#include <map>
#include <set>

#include <cob_3d_mapping_msgs/Shape.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>


namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace ogre_tools
{
class Object;
}

namespace rviz
{

class MarkerSelectionHandler;
typedef boost::shared_ptr<MarkerSelectionHandler> MarkerSelectionHandlerPtr;

class ShapeBase;
typedef boost::shared_ptr<ShapeBase> ShapeBasePtr;

typedef std::pair<std::string, int32_t> MarkerID;

/**
 * \class ShapeDisplay
 * \brief Displays "markers" sent in by other ROS nodes on the "visualization_marker" topic
 *
 * Markers come in as cob_3d_mapping_msgs::Shape messages.  See the Marker message for more information.
 */
class ShapeDisplay : public Display
{
public:
  ShapeDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~ShapeDisplay();

  virtual void update(float wall_dt, float ros_dt);

  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();
  virtual void reset();

  void setMarkerTopic(const std::string& topic);
  const std::string& getMarkerTopic() { return marker_topic_; }

  virtual void createProperties();

  void setNamespaceEnabled(const std::string& ns, bool enabled);
  bool isNamespaceEnabled(const std::string& ns);

  void deleteMarker(MarkerID id);

  void setMarkerStatus(MarkerID id, StatusLevel level, const std::string& text);
  void deleteMarkerStatus(MarkerID id);

protected:
  virtual void onEnable();
  virtual void onDisable();

  /**
   * \brief Subscribes to the "visualization_marker" and "visualization_marker_array" topics
   */
  virtual void subscribe();
  /**
   * \brief Unsubscribes from the "visualization_marker" "visualization_marker_array" topics
   */
  virtual void unsubscribe();

  /**
   * \brief Removes all the markers
   */
  void clearMarkers();

  /**
   * \brief Processes a marker message
   * @param message The message to process
   */
  void processMessage( const cob_3d_mapping_msgs::Shape::ConstPtr& message );
  /**
   * \brief Processes an "Add" marker message
   * @param message The message to process
   */
  void processAdd( const cob_3d_mapping_msgs::Shape::ConstPtr& message );
  /**
   * \brief Processes a "Delete" marker message
   * @param message The message to process
   */
  void processDelete( const cob_3d_mapping_msgs::Shape::ConstPtr& message );

  //MarkerBasePtr getMarker(MarkerID id);

  /**
   * \brief ROS callback notifying us of a new marker
   */
  void incomingMarker(const cob_3d_mapping_msgs::Shape::ConstPtr& marker);

  void incomingMarkerArray(const cob_3d_mapping_msgs::ShapeArray::ConstPtr& array);

  void failedMarker(const cob_3d_mapping_msgs::Shape::ConstPtr& marker, tf::FilterFailureReason reason);

  typedef std::vector<ShapeBasePtr> M_IDToMarker;
  typedef std::set<ShapeBasePtr> S_MarkerBase;
  M_IDToMarker markers_;                                ///< Map of marker id to the marker info structure
  //S_MarkerBase markers_with_expiration_;
  //S_MarkerBase frame_locked_markers_;
  typedef std::vector<cob_3d_mapping_msgs::Shape::ConstPtr> V_MarkerMessage;
  V_MarkerMessage message_queue_;                       ///< Marker message queue.  Messages are added to this as they are received, and then processed
                                                        ///< in our update() function
  boost::mutex queue_mutex_;

  Ogre::SceneNode* scene_node_;                         ///< Scene node all the marker objects are parented to

  message_filters::Subscriber<cob_3d_mapping_msgs::Shape> sub_;
  tf::MessageFilter<cob_3d_mapping_msgs::Shape> tf_filter_;
  ros::Subscriber array_sub_;

  std::string marker_topic_;

  struct Namespace
  {
    std::string name;
    bool enabled;
    BoolPropertyWPtr prop;
  };
  typedef std::map<std::string, Namespace> M_Namespace;
  M_Namespace namespaces_;

  ROSTopicStringPropertyWPtr marker_topic_property_;
  CategoryPropertyWPtr namespaces_category_;
};

} // namespace rviz

#endif /* SHAPE_DISPLAY_H */
