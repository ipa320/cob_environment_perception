/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *
 * Author: Adam Leeper
 */

#ifndef RVIZ_FREE_VIEW_CONTROLLER_H
#define RVIZ_FREE_VIEW_CONTROLLER_H

#include "rviz_view_controllers/CameraPlacementTrajectory.h"
#include "rviz/view_controller.h"

#include <ros/subscriber.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace rviz
{

class SceneNode;
class Shape;
class BoolProperty;
class FloatProperty;
class VectorProperty;
class QuaternionProperty;
class TfFrameProperty;
class EditableEnumProperty;

/** @brief An un-constrained "flying" camera, controlled by a position and quaternion. */
class FreeViewController : public ViewController
{
Q_OBJECT
public:

  enum { TRANSITION_LINEAR = 0,
         TRANSITION_SPHERICAL};

  FreeViewController();
  virtual ~FreeViewController();

  virtual void onInitialize();

  /** @brief called by activate().
   *
   * Override to implement view-specific activation.  This version
   * calls updateTargetSceneNode(). */
  virtual void onActivate();

  void yaw( float angle );
  void pitch( float angle );
  void move( float x, float y, float z );
  void move_camera( float x, float y, float z );

  void yaw_pitch_roll( float yaw, float pitch, float roll );

  virtual void handleMouseEvent(ViewportMouseEvent& evt);

  virtual void lookAt( const Ogre::Vector3& point );
  void orbitCameraTo( const Ogre::Vector3& point);
  void moveCameraWithFocusTo( const Ogre::Vector3& point);

  virtual void reset();

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
  virtual void mimic( ViewController* source_view );

protected Q_SLOTS:
  /** @brief Called when Target Frame property changes while view is
   * active.  Purpose is to change values in the view controller (like
   * a position offset) such that the actual viewpoint does not
   * change.  Calls updateTargetSceneNode() and
   * onTargetFrameChanged(). */
  virtual void updateAttachedFrame();
  

  virtual void onDistancePropertyChanged();
  
  virtual void onFocusPropertyChanged();
  
  virtual void onEyePropertyChanged();

protected:

  virtual void update(float dt, float ros_dt);
  /** @brief Override to implement the change in properties which
   * nullifies the change in target frame.
   * @see updateTargetFrame() */
  virtual void onAttachedFrameChanged( const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation );

  /** @brief Update the position of the target_scene_node_ from the TF
   * frame specified in the Target Frame property. */
  void updateAttachedSceneNode();

  void cameraPlacementCallback(const rviz_view_controllers::CameraPlacementConstPtr &cp_ptr);
  void cameraPlacementTrajectoryCallback(const rviz_view_controllers::CameraPlacementTrajectoryConstPtr &cptptr);
  void transformCameraPlacementToAttachedFrame(rviz_view_controllers::CameraPlacement &cp);

  void setPropertiesFromCamera( Ogre::Camera* source_camera );

  // Begin a camera movement animation
  void beginNewTransition(const Ogre::Vector3 &camera, const Ogre::Vector3 &focus, const Ogre::Vector3 &up,
                          const ros::Duration &transition_time);
  void cancelTransition();

  void updateCamera();

  Ogre::Vector3 fixedFrameToAttachedLocal(const Ogre::Vector3 &v) { return reference_orientation_.Inverse()*(v - reference_position_); }
  Ogre::Vector3 attachedLocalToFixedFrame(const Ogre::Vector3 &v) { return reference_position_ + (reference_orientation_*v); }

  float getDistanceFromCameraToFocalPoint(); ///< Return the distance between camera and focal point.

  Ogre::Quaternion getOrientation(); ///< Return a Quaternion

  BoolProperty* interaction_enabled_property_;  ///< If True, most changes to camera state are disabled.
  EditableEnumProperty* interaction_mode_property_;  ///< Select between Orbit or FPS control style.
  BoolProperty* fixed_up_property_;             ///< If True, "up" is fixed to ... up.

  FloatProperty* distance_property_;            ///< The camera's distance from the focal point
  VectorProperty* eye_point_property_;           ///< The position of the camera.
  VectorProperty* focus_point_property_;        ///< The point around which the camera "orbits".
  VectorProperty* up_vector_property_;        ///< The up vector for the camera.
  FloatProperty* default_transition_time_property_;

  Ogre::Vector3 start_position_, goal_position_;
  Ogre::Vector3 start_focus_, goal_focus_;
  Ogre::Vector3 start_up_, goal_up_;
  ros::Time trajectory_start_time_;
  ros::Time transition_start_time_;
  ros::Duration current_transition_duration_;

  Shape* focal_shape_;
  bool dragging_;
  bool animate_;

  QCursor interaction_disabled_cursor_;         ///< A cursor for indicating mouse interaction is disabled.
  
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber placement_subscriber_;

  TfFrameProperty* attached_frame_property_;
  Ogre::SceneNode* attached_scene_node_;
  
  Ogre::Quaternion reference_orientation_;
  Ogre::Vector3 reference_position_;

};

} // end namespace rviz

#endif // RVIZ_VIEW_CONTROLLER_H
