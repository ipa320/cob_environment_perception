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

#include "cob_3d_mapping_rviz_plugins/shape_marker.h"
#include "cob_3d_mapping_rviz_plugins/polypartition.h"
#include "rviz/default_plugin/markers/marker_selection_handler.h"
#include "cob_3d_mapping_rviz_plugins/shape_display.h"

#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreMatrix3.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transform.h>

#include <sensor_msgs/PointCloud2.h>

namespace rviz
{

  ShapeMarker::ShapeMarker( ShapeDisplay* owner, VisualizationManager* manager,
                            Ogre::SceneNode* parent_node ) :
                            ShapeBase(owner, manager, parent_node), polygon_(0)
  {
    scene_node_ = manager->getSceneManager()->getRootSceneNode()->createChildSceneNode();

    static int count = 0;
    std::stringstream ss;
    ss << "Polygon" << count++;
    polygon_ = manager->getSceneManager()->createManualObject( ss.str() );
    polygon_->setDynamic( true );
    scene_node_->attachObject( polygon_ );
  }

  ShapeMarker::~ShapeMarker()
  {
    delete polygon_;
  }

  void ShapeMarker::onNewMessage( const MarkerConstPtr& old_message,
                                  const MarkerConstPtr& new_message )
  {
    Eigen::Vector3f u,v,normal,origin;
    Eigen::Affine3f transformation;
    normal(0)=new_message->params[0];
    normal(1)=new_message->params[1];
    normal(2)=new_message->params[2];
    origin(0)=new_message->params[4];
    origin(1)=new_message->params[5];
    origin(2)=new_message->params[6];
    //std::cout << "normal: " << normal << std::endl;
    //std::cout << "centroid: " << origin << std::endl;
    v = normal.unitOrthogonal ();
    u = normal.cross (v);
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal,  origin, transformation);

    TPPLPartition pp;
    list<TPPLPoly> polys,result;

    //fill polys
    for(size_t i=0; i<1/*new_message->points.size()*/; i++) {
      pcl::PointCloud<pcl::PointXYZ> pc;
      TPPLPoly poly;

      pcl::fromROSMsg(new_message->points[i],pc);

      poly.Init(pc.size());
      poly.SetHole(i>0);

      for(size_t j=0; j<pc.size(); j++) {
        //std::cout << "before:" << pc[j].x << "," << pc[j].y << "," << pc[j].z << std::endl;
        Eigen::Vector3f p3 = transformation*pc[j].getVector3fMap();
        poly[j].x = p3(0);
        poly[j].y = p3(1);
        //std::cout << "after:" << p3(0) << "," << p3(1) << "," << p3(2) << std::endl;
      }
      poly.SetOrientation(TPPL_CCW);

      polys.push_back(poly);
    }

    pp.Triangulate_EC(&polys,&result);


    polygon_->clear();
    polygon_->begin("RVIZ/Blue", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    Ogre::ColourValue color( 0, 0, 1, 1 );
    TPPLPoint p1;

    transformation=transformation.inverse();
    for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
      //draw each triangle
      //ROS_INFO("number %d", it->GetNumPoints());
      for(size_t i=0;i<it->GetNumPoints();i++) {
        p1 = it->GetPoint(i);

        Eigen::Vector3f p3;
        p3(0)=p1.x;
        p3(1)=p1.y;
        p3(2)=0;
        p3 = transformation*p3;

        //std::cout << p3(0) << "," << p3(1) << "," << p3(2) << std::endl;

        polygon_->position(p3(0),p3(1),p3(2));  // start position
        polygon_->normal(normal(0),normal(1),normal(2));
        polygon_->colour(color);
      }
    }

    for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
      //draw each triangle
      //ROS_INFO("number %d", it->GetNumPoints());
      for(int i=it->GetNumPoints()-1;i>=0;i--) {
        p1 = it->GetPoint(i);

        Eigen::Vector3f p3;
        p3(0)=p1.x;
        p3(1)=p1.y;
        p3(2)=0;
        p3 = transformation*p3;

        //std::cout << p3(0) << "," << p3(1) << "," << p3(2) << std::endl;

        polygon_->position(p3(0),p3(1),p3(2));  // start position
        polygon_->colour(color);
      }
    }

    polygon_->end();


    vis_manager_->getSelectionManager()->removeObject(coll_);
    /*coll_ = vis_manager_->getSelectionManager()->createCollisionForObject(
        shape_, SelectionHandlerPtr(new MarkerSelectionHandler(this, MarkerID(
            "fake_ns", new_message->id))), coll_);*/

    Ogre::Vector3 pos, scale, scale_correct;
    Ogre::Quaternion orient;
    //transform(new_message, pos, orient, scale);

    /*if (owner_ && (new_message->scale.x * new_message->scale.y
     * new_message->scale.z == 0.0f))
  {
    owner_->setMarkerStatus(getID(), status_levels::Warn,
        "Scale of 0 in one of x/y/z");
  }*/
    pos.x = new_message->params[4];
    pos.y = new_message->params[5];
    pos.z = new_message->params[6];

    setPosition(pos);
    setOrientation( orient * Ogre::Quaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ) );

    //scale_correct = Ogre::Quaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ) * scale;

    //shape_->setScale(scale_correct);
  }

  S_MaterialPtr ShapeMarker::getMaterials()
  {
    S_MaterialPtr materials;
    return materials;
  }

}
