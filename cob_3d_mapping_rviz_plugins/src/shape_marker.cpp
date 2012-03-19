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

  std::string createMaterialIfNotExists(const float r, const float g, const float b, const float a)
  {
    char buf[128];
    sprintf(buf, "ShapeColor%f;%f;%f;%f",r,g,b,a);
    if(!Ogre::MaterialManager::getSingleton().getByName(buf, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME).isNull())
      return buf;

    Ogre::ColourValue color( r,g,b,a );
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create( buf, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    mat->setAmbient(color * 0.01f);
    mat->setDiffuse(color);
    mat->setLightingEnabled(true);
    mat->setReceiveShadows(true);
    mat->setCullingMode(Ogre::CULL_NONE);

    return buf;
  }

  TPPLPoint MsgToPoint2D(const pcl::PointXYZ &point, const cob_3d_mapping_msgs::Shape::ConstPtr& new_message) {
    TPPLPoint pt;

    if(new_message->params.size()==4) {
      Eigen::Vector3f u,v,normal,origin;
      Eigen::Affine3f transformation;
      normal(0)=new_message->params[0];
      normal(1)=new_message->params[1];
      normal(2)=new_message->params[2];
      origin(0)=new_message->centroid.x;
      origin(1)=new_message->centroid.y;
      origin(2)=new_message->centroid.z;
      //std::cout << "normal: " << normal << std::endl;
      //std::cout << "centroid: " << origin << std::endl;
      v = normal.unitOrthogonal ();
      u = normal.cross (v);
      pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal,  origin, transformation);

      Eigen::Vector3f p3 = transformation*point.getVector3fMap();
      pt.x = p3(0);
      pt.y = p3(1);
    }
    else if(new_message->params.size()==5) {
      pt.x=point.x;
      pt.y=point.y;
    }

    return pt;
  }

  void MsgToPoint3D(const TPPLPoint &pt, const cob_3d_mapping_msgs::Shape::ConstPtr& new_message, Eigen::Vector3f &pos, Eigen::Vector3f &normal) {
    if(new_message->params.size()==4) {
      Eigen::Vector3f u,v,origin;
      Eigen::Affine3f transformation;
      normal(0)=new_message->params[0];
      normal(1)=new_message->params[1];
      normal(2)=new_message->params[2];
      origin(0)=new_message->centroid.x;
      origin(1)=new_message->centroid.y;
      origin(2)=new_message->centroid.z;
      //std::cout << "normal: " << normal << std::endl;
      //std::cout << "centroid: " << origin << std::endl;
      v = normal.unitOrthogonal ();
      u = normal.cross (v);
      pcl::getTransformationFromTwoUnitVectorsAndOrigin(v, normal,  origin, transformation);

      transformation=transformation.inverse();

      Eigen::Vector3f p3;
      p3(0)=pt.x;
      p3(1)=pt.y;
      p3(2)=0;
      pos = transformation*p3;
    }
    else if(new_message->params.size()==5) {
      Eigen::Vector2f v,v2,n2;
      v(0)=pt.x;
      v(1)=pt.y;
      v2=v;
      v2(0)*=v2(0);
      v2(1)*=v2(1);
      n2(0)=new_message->params[3];
      n2(1)=new_message->params[4];

      //dummy normal
      normal(0)=new_message->params[0];
      normal(1)=new_message->params[1];
      normal(2)=new_message->params[2];

      Eigen::Vector3f x,y, origin;
      x(0)=1.f;
      y(1)=1.f;
      x(1)=x(2)=y(0)=y(2)=0.f;

      Eigen::Matrix<float,3,2> proj2plane_;
      proj2plane_.col(0)=normal.cross(y);
      proj2plane_.col(1)=normal.cross(x);

      origin(0)=new_message->centroid.x;
      origin(1)=new_message->centroid.y;
      origin(2)=new_message->centroid.z;

      pos = origin+proj2plane_*v + normal*(v2.dot(n2));
      normal += normal*(v).dot(n2);
    }
  }

  Eigen::Vector3f MsgToOrigin(const cob_3d_mapping_msgs::Shape::ConstPtr& new_message) {
    Eigen::Vector3f origin;

    origin(0)=origin(1)=origin(2)=0.f;

    if(new_message->params.size()==4) {
      origin(0)=new_message->centroid.x;
      origin(1)=new_message->centroid.y;
      origin(2)=new_message->centroid.z;
    }

    return origin;
  }

  void ShapeMarker::onNewMessage( const MarkerConstPtr& old_message,
                                  const MarkerConstPtr& new_message )
  {

    TPPLPartition pp;
    list<TPPLPoly> polys,result;

    //fill polys
    for(size_t i=0; i<new_message->points.size(); i++) {
      pcl::PointCloud<pcl::PointXYZ> pc;
      TPPLPoly poly;

      pcl::fromROSMsg(new_message->points[i],pc);

      poly.Init(pc.size());
      poly.SetHole(new_message->holes[i]);

      for(size_t j=0; j<pc.size(); j++) {
        poly[j] = MsgToPoint2D(pc[j], new_message);
      }
      if(new_message->holes[i])
        poly.SetOrientation(TPPL_CW);
      else
        poly.SetOrientation(TPPL_CCW);

      polys.push_back(poly);
    }

    pp.Triangulate_EC(&polys,&result);


    polygon_->clear();
    polygon_->begin(createMaterialIfNotExists(new_message->color.r,new_message->color.b,new_message->color.g,new_message->color.a), Ogre::RenderOperation::OT_TRIANGLE_LIST);

    TPPLPoint p1;

    for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
      //draw each triangle
      for(size_t i=0;i<it->GetNumPoints();i++) {
        p1 = it->GetPoint(i);

        Eigen::Vector3f p3, normal;
        MsgToPoint3D(p1,new_message,p3,normal);

        polygon_->position(p3(0),p3(1),p3(2));  // start position
        polygon_->normal(normal(0),normal(1),normal(2));
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
    Eigen::Vector3f origin=MsgToOrigin(new_message);
    pos.x = origin(0);
    pos.y = origin(1);
    pos.z = origin(2);

    //setPosition(pos);
    return;
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
