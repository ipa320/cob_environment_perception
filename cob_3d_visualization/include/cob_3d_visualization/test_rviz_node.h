/*
 * test_rviz_node.h
 *
 *  Created on: Nov 19, 2012
 *      Author: goa-sn
 */

#ifndef TEST_RVIZ_NODE_H_
#define TEST_RVIZ_NODE_H_
/*
 * get_rviz_ogre_camera.h
 *
 *  Created on: Nov 16, 2012
 *      Author: goa-sn
 */

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
#include <geometry_msgs/Pose.h>

namespace rviz
{
  class SceneNode;

  class TestRvizNode : public ViewController , public Display
  {
    public:
      TestRvizNode(VisualizationManager* manager, const std::string& name);
      ~TestRvizNode() {}

      void testFunc();

    protected:
      Ogre::SceneNode* scene_node_;                         ///< Scene node all the marker objects are parented to
      //
      ros::Publisher cam_pose_pub_ ;
      ros::NodeHandle nh_ ;

  };
}

#endif /* TEST_RVIZ_NODE_H_ */
