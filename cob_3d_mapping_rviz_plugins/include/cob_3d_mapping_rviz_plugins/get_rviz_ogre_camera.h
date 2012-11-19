/*
 * get_rviz_ogre_camera.h
 *
 *  Created on: Nov 16, 2012
 *      Author: goa-sn
 */

#ifndef GET_RVIZ_OGRE_CAMERA_H_
#define GET_RVIZ_OGRE_CAMERA_H_

#include <ros/ros.h>
/*OGRE Includes*/
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
/*rviz Includes*/
#include "rviz/view_controller.h"
#include "rviz/visualization_manager.h"
#include <rviz/render_panel.h>
#include "rviz/display.h"

namespace rviz
{
  class SceneNode;

  class GetRvizCameraView : public ViewController , public Display
  {
    public:
      GetRvizCameraView(VisualizationManager* manager, const std::string& name);
      ~GetRvizCameraView() {}

      void testFunc();

    protected:
      Ogre::SceneNode* scene_node_;                         ///< Scene node all the marker objects are parented to
      //
      ros::Publisher cam_pose_pub_ ;
      ros::NodeHandle nh_ ;

  };
}
#endif /* GET_RVIZ_OGRE_CAMERA_H_ */
