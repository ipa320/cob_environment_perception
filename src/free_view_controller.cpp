/*
 * Copyright (c) 2009, Willow Garage, Inc.
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


#include "rviz_view_controllers/free_view_controller.h"

#include "rviz/load_resource.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/display_context.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/frame_manager.h"
#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/editable_enum_property.h"
#include "rviz/properties/ros_topic_property.h"

#include <OGRE/OgreViewport.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>

namespace rviz
{
using namespace rviz_view_controllers;

// Strings for selecting control mode styles
static const std::string MODE_ORBIT = "Orbit";
static const std::string MODE_FPS = "FPS";

// Limits to prevent orbit controller singularity, but not currently used.
static const float PITCH_LIMIT_LOW = -Ogre::Math::HALF_PI + 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::HALF_PI - 0.001;

// Some convenience functions for Ogre / geometry_msgs conversions
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::Point &m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::Vector3 &m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline geometry_msgs::Point pointOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Point m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}
static inline void pointOgreToMsg(const Ogre::Vector3 &o, geometry_msgs::Point &m)  { m.x = o.x; m.y = o.y; m.z = o.z; }

static inline geometry_msgs::Vector3 vectorOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Vector3 m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}
static inline void vectorOgreToMsg(const Ogre::Vector3 &o, geometry_msgs::Vector3 &m) { m.x = o.x; m.y = o.y; m.z = o.z; }

// -----------------------------------------------------------------------------


FreeViewController::FreeViewController()
  : nh_(""), animate_(false), dragging_( false )
{
  interaction_disabled_cursor_ = makeIconCursor( "package://rviz/icons/forbidden.png" );

  mouse_enabled_property_ = new BoolProperty("Mouse Enabled", true,
                                   "Enables mouse control of the camera.",
                                   this);
  interaction_mode_property_ = new EditableEnumProperty("Control Mode", QString::fromStdString(MODE_ORBIT),
                                   "Select the style of mouse interaction.",
                                   this);
  interaction_mode_property_->addOptionStd(MODE_ORBIT);
  interaction_mode_property_->addOptionStd(MODE_FPS);
  interaction_mode_property_->setStdString(MODE_ORBIT);

  fixed_up_property_ = new BoolProperty( "Maintain Vertical Axis", true,
                                         "If enabled, the camera is not allowed to roll side-to-side.",
                                          this);
  attached_frame_property_ = new TfFrameProperty("Camera-Fixed Frame",
                                                 TfFrameProperty::FIXED_FRAME_STRING,
                                                 "TF frame the camera is attached to.",
                                                 this, NULL, true );
  eye_point_property_    = new VectorProperty( "Eye", Ogre::Vector3( 5, 5, 10 ),
                                              "Position of the camera.", this );
  focus_point_property_ = new VectorProperty( "Focus", Ogre::Vector3::ZERO,
                                              "Position of the focus/orbit point.", this );
  distance_property_    = new FloatProperty( "Distance", getDistanceFromCameraToFocalPoint(),
                                             "The distance between the camera position and the focus point.",
                                             this );
  distance_property_->setMin( 0.01 );
  default_transition_time_property_ = new FloatProperty( "Transition Time", 0.5,
                                                         "The default time to use for camera transitions.",
                                                         this );
  camera_placement_topic_property_ = new RosTopicProperty("Placement Topic", "/rviz/camera_placement",
                                                          QString::fromStdString(ros::message_traits::datatype<rviz_view_controllers::CameraPlacement>() ),
                                                          "Topic for CameraPlacement messages", this, SLOT(updateTopics()));

  camera_placement_trajectory_topic_property_ = new RosTopicProperty("Trajectory Topic", "/rviz/camera_placement_trajectory",
                                                          QString::fromStdString(ros::message_traits::datatype<rviz_view_controllers::CameraPlacementTrajectory>() ),
                                                          "Topic for CameraPlacementTrajectory messages", this, SLOT(updateTopics()));
}

FreeViewController::~FreeViewController()
{
    delete focal_shape_;
    context_->getSceneManager()->destroySceneNode( attached_scene_node_ );
}

void FreeViewController::updateTopics()
{
  trajectory_subscriber_ = nh_.subscribe<rviz_view_controllers::CameraPlacementTrajectory>
                              (camera_placement_trajectory_topic_property_->getStdString(), 1,
                              boost::bind(&FreeViewController::cameraPlacementTrajectoryCallback, this, _1));
  placement_subscriber_  = nh_.subscribe<rviz_view_controllers::CameraPlacement>
                              (camera_placement_topic_property_->getStdString(), 1,
                              boost::bind(&FreeViewController::cameraPlacementCallback, this, _1));
}

void FreeViewController::onInitialize()
{
    attached_frame_property_->setFrameManager( context_->getFrameManager() );
    attached_scene_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    camera_->detachFromParent();
    attached_scene_node_->attachObject( camera_ );

    camera_->setProjectionType( Ogre::PT_PERSPECTIVE );

    focal_shape_ = new Shape(Shape::Sphere, context_->getSceneManager(), attached_scene_node_);
    focal_shape_->setScale(Ogre::Vector3(0.05f, 0.05f, 0.01f));
    focal_shape_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
    focal_shape_->getRootNode()->setVisible(false);

    updateTopics();
}

void FreeViewController::onActivate()
{
  updateAttachedSceneNode();

  // Before activation, changes to target frame property should have
  // no side-effects.  After activation, changing target frame
  // property has the side effect (typically) of changing an offset
  // property so that the view does not jump.  Therefore we make the
  // signal/slot connection from the property here in onActivate()
  // instead of in the constructor.
  connect( attached_frame_property_, SIGNAL( changed() ), this, SLOT( updateAttachedFrame() ));
  connect( distance_property_, SIGNAL( changed() ), this, SLOT( onDistancePropertyChanged() ));
  connect( eye_point_property_, SIGNAL( changed() ), this, SLOT( onEyePropertyChanged() ));
  connect( focus_point_property_, SIGNAL( changed() ), this, SLOT( onFocusPropertyChanged() ));
}

void FreeViewController::onEyePropertyChanged()
{
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

void FreeViewController::onFocusPropertyChanged()
{
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

void FreeViewController::onDistancePropertyChanged()
{
  // Uncommenting this causes annoying loops with the functions above.
  //Ogre::Vector3 new_eye_position = focus_point_property_->getVector() + distance_property_->getFloat()* camera_->getOrientation().zAxis();
  //eye_point_property_->setVector(new_eye_position);
}
void FreeViewController::updateAttachedFrame()
{
  Ogre::Vector3 old_position = attached_scene_node_->getPosition();
  Ogre::Quaternion old_orientation = attached_scene_node_->getOrientation();

  updateAttachedSceneNode();

  onAttachedFrameChanged( old_position, old_orientation );
}

void FreeViewController::updateAttachedSceneNode()
{
  Ogre::Vector3 new_reference_position;
  Ogre::Quaternion new_reference_orientation;

  bool queue = false;
  if( context_->getFrameManager()->getTransform( attached_frame_property_->getFrameStd(), ros::Time(),
                                                 new_reference_position, new_reference_orientation ))
  {
    attached_scene_node_->setPosition( new_reference_position );
    attached_scene_node_->setOrientation( new_reference_orientation );
    reference_position_ = new_reference_position;
    reference_orientation_ = new_reference_orientation;
    queue = true;
  }
  if(queue) context_->queueRender();
}

void FreeViewController::onAttachedFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  Ogre::Vector3 fixed_frame_focus_position = old_reference_orientation*focus_point_property_->getVector() + old_reference_position;
  Ogre::Vector3 fixed_frame_eye_position = old_reference_orientation*eye_point_property_->getVector() + old_reference_position;
  Ogre::Vector3 new_focus_position = fixedFrameToAttachedLocal(fixed_frame_focus_position);
  Ogre::Vector3 new_eye_position = fixedFrameToAttachedLocal(fixed_frame_eye_position);

  Ogre::Quaternion new_camera_orientation = reference_orientation_.Inverse()*old_reference_orientation*getOrientation();

  focus_point_property_->setVector(new_focus_position);
  eye_point_property_->setVector(new_eye_position);
  camera_->setOrientation(new_camera_orientation);

}

float FreeViewController::getDistanceFromCameraToFocalPoint()
{
    return (eye_point_property_->getVector() - focus_point_property_->getVector()).length();
}

void FreeViewController::reset()
{
    eye_point_property_->setVector(Ogre::Vector3(5, 5, 10));
    focus_point_property_->setVector(Ogre::Vector3::ZERO);
    distance_property_->setFloat( getDistanceFromCameraToFocalPoint());
    mouse_enabled_property_->setBool(true);
    interaction_mode_property_->setStdString(MODE_ORBIT);


  // Hersh says: why is the following junk necessary?  I don't know.
  // However, without this you need to call reset() twice after
  // switching from TopDownOrtho to FPS.  After the first call the
  // camera is in the right position but pointing the wrong way.
  updateCamera();
  camera_->lookAt( 0, 0, 0 );
  setPropertiesFromCamera( camera_ );
}

void FreeViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if( !mouse_enabled_property_->getBool() )
  {
    setCursor( interaction_disabled_cursor_ );
    return;
  }

  float distance = distance_property_->getFloat();
  int32_t diff_x = 0;
  int32_t diff_y = 0;
  bool moved = false;

  if( event.type == QEvent::MouseButtonPress )
  {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
    dragging_ = true;
    cancelTransition();  // Stop any automated movement
  }
  else if( event.type == QEvent::MouseButtonRelease )
  {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
    dragging_ = false;
  }
  else if( dragging_ && event.type == QEvent::MouseMove )
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  // regular left-button drag
  if( event.left() && !event.shift() )
  {
    setCursor( Rotate3D );
    yaw_pitch_roll( -diff_x*0.005, -diff_y*0.005, 0 );
  }
  // middle or shift-left drag
  else if( event.middle() || ( event.shift() && event.left() ))
  {
    setCursor( MoveXY );
    if(interaction_mode_property_->getStdString() == MODE_ORBIT)  // Orbit style
    {
        float fovY = camera_->getFOVy().valueRadians();
        float fovX = 2.0f * atan( tan( fovY / 2.0f ) * camera_->getAspectRatio() );

        int width = camera_->getViewport()->getActualWidth();
        int height = camera_->getViewport()->getActualHeight();

        move_focus_and_eye( -((float)diff_x / (float)width) * distance * tan( fovX / 2.0f ) * 2.0f,
              ((float)diff_y / (float)height) * distance * tan( fovY / 2.0f ) * 2.0f,
              0.0f );
    }
    else if(interaction_mode_property_->getStdString() == MODE_FPS)  // Orbit style
    {
      move_focus_and_eye( diff_x*0.01, -diff_y*0.01, 0.0f );
    }
  }
  else if( event.right() )
  {
    if( event.shift() ||  (interaction_mode_property_->getStdString() == MODE_FPS) )
    {
      setCursor( MoveZ );
      move_focus_and_eye(0.0f, 0.0f, diff_y * 0.1 * (distance / 10.0f));
    }
    else
    {
      setCursor( Zoom );
      move_eye( 0, 0, diff_y * 0.1 * (distance / 10.0f) );
    }
  }
  else
  {
    setCursor( event.shift() ? MoveXY : Rotate3D );
  }

  if ( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;
    yaw_pitch_roll(0, 0, diff*0.001 );

    moved = true;
  }

  if(event.type == QEvent::MouseButtonPress && event.left() && event.control())
  {
    bool was_orbit = (interaction_mode_property_->getStdString() == MODE_ORBIT);
    interaction_mode_property_->setStdString(was_orbit ? MODE_FPS : MODE_ORBIT );
  }

  if (moved)
  {
    context_->queueRender();
  }
}

void FreeViewController::setPropertiesFromCamera( Ogre::Camera* source_camera )
{
  Ogre::Vector3 direction = source_camera->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
  eye_point_property_->setVector( source_camera->getPosition() );
  focus_point_property_->setVector( source_camera->getPosition() + direction*distance_property_->getFloat());
}

void FreeViewController::mimic( ViewController* source_view )
{
    QVariant target_frame = source_view->subProp( "Target Frame" )->getValue();
    if( target_frame.isValid() )
    {
      attached_frame_property_->setValue( target_frame );
    }

    Ogre::Camera* source_camera = source_view->getCamera();
    Ogre::Vector3 position = source_camera->getPosition();
    Ogre::Quaternion orientation = source_camera->getOrientation();

    if( source_view->getClassId() == "rviz/Orbit" )
    {
        distance_property_->setFloat( source_view->subProp( "Distance" )->getValue().toFloat() );
    }
    else
    {
        distance_property_->setFloat( position.length() );
    }
    interaction_mode_property_->setStdString( MODE_ORBIT );

    Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_property_->getFloat() );
    focus_point_property_->setVector( position + direction );
    eye_point_property_->setVector(position);
    updateCamera();
}

void FreeViewController::beginNewTransition(const Ogre::Vector3 &eye, const Ogre::Vector3 &focus, const Ogre::Vector3 &up,
                                            const ros::Duration &transition_time)
{
  if(ros::Duration(transition_time).isZero())
  {
    eye_point_property_->setVector(eye);
    focus_point_property_->setVector(focus);
    distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
    return;
  }

  start_position_ = eye_point_property_->getVector();
  goal_position_ = eye;

  start_focus_ = focus_point_property_->getVector();
  goal_focus_ = focus;

  start_up_ = getOrientation().yAxis();
  goal_up_ = up;

  current_transition_duration_ = ros::Duration(transition_time);
  transition_start_time_ = ros::Time::now();

  animate_ = true;
}

void FreeViewController::cancelTransition()
{
  animate_ = false;
}

void FreeViewController::cameraPlacementCallback(const CameraPlacementConstPtr &cp_ptr)
{
  CameraPlacement cp = *cp_ptr;

  if(cp.attached_frame != "")
  {
    attached_frame_property_->setStdString(cp.attached_frame);
    updateAttachedFrame();
  }

  ROS_DEBUG_STREAM("Received a camera placement request! \n" << cp);
  transformCameraPlacementToAttachedFrame(cp);
  ROS_DEBUG_STREAM("After transform, we have \n" << cp);

  Ogre::Vector3 eye = vectorFromMsg(cp.eye.point); 
  Ogre::Vector3 focus = vectorFromMsg(cp.focus.point); 
  Ogre::Vector3 up = vectorFromMsg(cp.up.vector); 

  beginNewTransition(eye, focus, up, cp.time_from_start);
}

void FreeViewController::cameraPlacementTrajectoryCallback(const CameraPlacementTrajectoryConstPtr &cptptr)
{
  CameraPlacementTrajectory cpt = *cptptr;
  ROS_DEBUG_STREAM("Received a camera placement trajectory request! \n" << cpt);
  
  // Handle control parameters
  mouse_enabled_property_->setBool( cpt.interaction_enabled );
  fixed_up_property_->setBool( cpt.maintain_fixed_up_axis );
  if(cpt.mouse_interaction_mode != cpt.NO_CHANGE)
  {
    std::string name = "";
    if(cpt.mouse_interaction_mode == cpt.ORBIT) name = MODE_ORBIT;
    else if(cpt.mouse_interaction_mode == cpt.FPS) name = MODE_FPS;
    interaction_mode_property_->setStdString(name);
  }

  // TODO should transform the interpolated positions (later), or transform info will only reflect the TF tree state at the beginning...
  for(size_t i = 0; i<cpt.placements.size(); i++)
  {
    transformCameraPlacementToAttachedFrame(cpt.placements[i]);
  }

  // For now, just transition to the first placement until we put in the capacity for a trajectory
  CameraPlacement cp = cpt.placements[0];
  Ogre::Vector3 eye = vectorFromMsg(cp.eye.point); 
  Ogre::Vector3 focus = vectorFromMsg(cp.focus.point); 
  Ogre::Vector3 up = vectorFromMsg(cp.up.vector); 

  beginNewTransition(eye, focus, up, cp.time_from_start);
}

void FreeViewController::transformCameraPlacementToAttachedFrame(CameraPlacement &cp)
{
//  geometry_msgs::Pose eye_pose, focus_pose;
//  eye_pose.position = cp.camera.point;
//  focus_pose.position = cp.focus.point;
//  Ogre::Vector3 eye, focus;
//  // If I want to handle the up vector I need to pack it into the quaternion, since the vectors are translated.
//  // No!! Bad!! Ogre::Vector3 up(cp.up.point.x, cp.up.point.y, cp.up.point.z);
//  Ogre::Quaternion eye_q, focus_q;
//  context_->getFrameManager()->transform(cp.camera.header.frame_id, ros::Time(0), eye_pose,   eye,   eye_q);
//  context_->getFrameManager()->transform(cp.focus.header.frame_id,  ros::Time(0), focus_pose, focus, focus_q);

  Ogre::Vector3 position_fixed_eye, position_fixed_focus, position_fixed_up; // position_fixed_attached;
  Ogre::Quaternion rotation_fixed_eye, rotation_fixed_focus, rotation_fixed_up; // rotation_fixed_attached;

  context_->getFrameManager()->getTransform(cp.eye.header.frame_id, ros::Time(0), position_fixed_eye, rotation_fixed_eye);
  context_->getFrameManager()->getTransform(cp.focus.header.frame_id,  ros::Time(0), position_fixed_focus, rotation_fixed_focus);
  context_->getFrameManager()->getTransform(cp.up.header.frame_id,  ros::Time(0), position_fixed_up, rotation_fixed_up);
  //context_->getFrameManager()->getTransform(attached_frame_property_->getStdString(),  ros::Time(0), position_fixed_attached, rotation_fixed_attached);

  Ogre::Vector3 eye = vectorFromMsg(cp.eye.point); 
  Ogre::Vector3 focus = vectorFromMsg(cp.focus.point); 
  Ogre::Vector3 up = vectorFromMsg(cp.up.vector); 

  eye = fixedFrameToAttachedLocal(position_fixed_eye + rotation_fixed_eye*eye);
  focus = fixedFrameToAttachedLocal(position_fixed_focus + rotation_fixed_focus*focus);
  up = reference_orientation_.Inverse()*rotation_fixed_up*up;

  cp.eye.point = pointOgreToMsg(eye);
  cp.focus.point = pointOgreToMsg(focus);
  cp.up.vector = vectorOgreToMsg(up);
  cp.eye.header.frame_id = attached_frame_property_->getStdString();
  cp.focus.header.frame_id = attached_frame_property_->getStdString();
  cp.up.header.frame_id = attached_frame_property_->getStdString();
}


// We must assume that this point is in the Rviz Fixed frame since it came from Rviz...
void FreeViewController::lookAt( const Ogre::Vector3& point )
{
  if( !mouse_enabled_property_->getBool() ) return;

  Ogre::Vector3 new_point = fixedFrameToAttachedLocal(point);

  beginNewTransition(eye_point_property_->getVector(), new_point,
                     getOrientation().yAxis(),
                     ros::Duration(default_transition_time_property_->getFloat()));

  //  // Just for easily testing the other movement styles:
  //  orbitCameraTo(point);
  //  moveCameraWithFocusTo(point);
}

void FreeViewController::orbitCameraTo( const Ogre::Vector3& point)
{
  beginNewTransition(point, focus_point_property_->getVector(),
                     getOrientation().yAxis(),
                     ros::Duration(default_transition_time_property_->getFloat()));
}

void FreeViewController::moveEyeWithFocusTo( const Ogre::Vector3& point)
{
  beginNewTransition(point, focus_point_property_->getVector() + (point - eye_point_property_->getVector()),
                     getOrientation().yAxis(),
                     ros::Duration(default_transition_time_property_->getFloat()));
}


void FreeViewController::update(float dt, float ros_dt)
{
  updateAttachedSceneNode();

  if(animate_)
  {
    ros::Duration time_from_start = ros::Time::now() - transition_start_time_;
    float progress = time_from_start.toSec()/current_transition_duration_.toSec();
    // make sure we get all the way there before turning off
    if(progress > 1.0f)
    {
      progress = 1.0f;
      animate_ = false;
    }
    Ogre::Vector3 new_position = start_position_ + progress*(goal_position_ - start_position_);
    Ogre::Vector3 new_focus  = start_focus_ + progress*(goal_focus_ - start_focus_);
    Ogre::Vector3 new_up  = start_up_ + progress*(goal_up_ - start_up_);
    eye_point_property_->setVector( new_position );
    focus_point_property_->setVector( new_focus );
    //
    camera_->setPosition( eye_point_property_->getVector() );
    camera_->setFixedYawAxis(true, new_up);
    camera_->setDirection(attached_scene_node_->getOrientation() * (focus_point_property_->getVector() - eye_point_property_->getVector()));
    //
    distance_property_->setFloat( getDistanceFromCameraToFocalPoint());
  }

  updateCamera();
}

void FreeViewController::updateCamera()
{
//  camera_->setPosition( eye_point_property_->getVector() + reference_position_);
//  camera_->setFixedYawAxis(fixed_up_property_->getBool(), orientation_target_scene_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
//  camera_->setDirection(orientation_target_scene_node_->getOrientation() * (focus_point_property_->getVector() - eye_point_property_->getVector()));
//  focal_shape_->setPosition( focus_point_property_->getVector() + reference_position_ );
  camera_->setPosition( eye_point_property_->getVector() );
  camera_->setFixedYawAxis(fixed_up_property_->getBool(), attached_scene_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
  camera_->setDirection(attached_scene_node_->getOrientation() * (focus_point_property_->getVector() - eye_point_property_->getVector()));
  focal_shape_->setPosition( focus_point_property_->getVector() );
}

void FreeViewController::yaw_pitch_roll( float yaw, float pitch, float roll )
{
  Ogre::Quaternion yaw_quat, pitch_quat, roll_quat;
  yaw_quat.FromAngleAxis( Ogre::Radian( yaw ), Ogre::Vector3::UNIT_Y );
  pitch_quat.FromAngleAxis( Ogre::Radian( pitch ), Ogre::Vector3::UNIT_X );
  roll_quat.FromAngleAxis( Ogre::Radian( roll ), Ogre::Vector3::UNIT_Z );
  Ogre::Quaternion new_camera_orientation;
  new_camera_orientation = camera_->getOrientation() *  yaw_quat * pitch_quat * roll_quat;

  if( interaction_mode_property_->getStdString() == MODE_ORBIT )
  {
    // In orbit mode the focal point stays fixed, so we need to compute the new camera position.
    Ogre::Vector3 new_eye_position = focus_point_property_->getVector() + distance_property_->getFloat()* new_camera_orientation.zAxis();
    eye_point_property_->setVector(new_eye_position);
    camera_->setOrientation( new_camera_orientation );  // this order doesn't mater for small angles... right?
  }
  else
  {
    // In FPS mode the camera stays fixed, so we can just apply the rotations and then rely on the property update to set the new focal point.
    camera_->setOrientation( new_camera_orientation );  // this order doesn't mater for small angles... right?
    setPropertiesFromCamera(camera_);
  }
}

Ogre::Quaternion FreeViewController::getOrientation()  // Do we need this?
{
  return camera_->getOrientation();
}

void FreeViewController::move_focus_and_eye( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  eye_point_property_->add( getOrientation() * translate );
  focus_point_property_->add( getOrientation() * translate );
}

void FreeViewController::move_eye( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  // Only update the camera position if it won't "pass through" the origin
  Ogre::Vector3 new_position = eye_point_property_->getVector() + getOrientation() * translate;
  if( (new_position - focus_point_property_->getVector()).length() > distance_property_->getMin() )
    eye_point_property_->setVector(new_position);
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}



} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( rviz, Free, rviz::FreeViewController, rviz::ViewController )
