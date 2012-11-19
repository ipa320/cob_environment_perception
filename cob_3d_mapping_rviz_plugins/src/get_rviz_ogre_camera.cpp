
#include "cob_3d_mapping_rviz_plugins/get_rviz_ogre_camera.h"
#include "rviz/visualization_manager.h"
//#include "rviz/render_panel.h"
#include "rviz/properties/property_manager.h"
#include "rviz/properties/property.h"
#include "rviz/tf_frame_property.h"

#include "rviz/selection/selection_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <ogre_tools/arrow.h>
#include <ogre_tools/shape.h>
#include <ogre_tools/billboard_line.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>


namespace rviz
{
  GetRvizCameraView ::GetRvizCameraView(VisualizationManager* manager, const std::string& name)
  :ViewController(manager,name,scene_node_), Display (name, manager)
  {
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    cam_pose_pub_ = nh_.advertise<geometry_msgs::Pose> ("rviz/cam_pose", 1);

    //    cam_pose_pub_ = nh_.advertise<geometry_msgs::Pose> ("rviz/cam_pose", 1);
    //    testFunc() ;
  }


  void GetRvizCameraView::testFunc() {


//    const Ogre::Vector3 &pose_ = scene_node_;
    const Ogre::Quaternion quat_ = scene_node_ ->getOrientation();



    //
    //    rviz::RenderPanel* render_panel_ = new rviz::RenderPanel()
    //    rviz::VisualizationManager* manager_ = new rviz::VisualizationManager(render_panel_);
    //
    //    render_panel_->initialize(scene_manager_, manager_);
    //
    //    manager_->initialize();
    //    manager_->startUpdate();
    //    std::cout << "running!!!!!!" << "\n" ;
    //    Ogre::Vector3    position = camera_->getPosition() ;
    //    Ogre::Quaternion quat     = camera_->getOrientation() ;

    //    geometry_msgs::Pose cam_pose;
    //
    //    cam_pose.position.x = position.x ;
    //    cam_pose.position.y = position.y ;
    //    cam_pose.position.z = position.z ;
    //
    //    cam_pose.orientation.x = quat.x ;
    //    cam_pose.orientation.y = quat.y ;
    //    cam_pose.orientation.z = quat.z ;
    //    cam_pose.orientation.w = quat.w ;


  }
}
