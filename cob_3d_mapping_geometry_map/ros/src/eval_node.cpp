//##################
//#### includes ####

// standard includes
//--
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
// ROS message includes
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_3d_mapping_msgs/GetGeometricMap.h>
#include <cob_3d_mapping_msgs/ShapeArray.h>


// external includes
#include <boost/timer.hpp>
#include "cob_3d_mapping_common/stop_watch.h"
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_mapping_common/polygon.h"


#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_mapping_common/cylinder.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


using namespace cob_3d_mapping;

class eval_node
{
public:

  eval_node()
  {
    seg_sub_ = n_.subscribe("seg_array", 10, &eval_node::seg_callback, this);
    map_sub_ = n_.subscribe("map_array", 10, &eval_node::map_callback, this);
    map_frame_id_="/map";
    era_ = 0;



  }

  ~eval_node()
  {
  }




  void
  map_callback(const cob_3d_mapping_msgs::ShapeArray::ConstPtr ma)
  {

    std::cout<<"TYP MAP SHAPE "<<ma->shapes[0].type<<std::endl;
    for(unsigned int i=0; i<ma->shapes.size(); i++)
    {

      if (ma->shapes[i].type == 0) {


        PolygonPtr polygon_map_entry_ptr = PolygonPtr(new Polygon());
        if(!fromROSMsg(ma->shapes[i], *polygon_map_entry_ptr)) {
          std::cout << "ERROR: fromROSMsg" << std::endl;
          continue;
        }



      }

      if (ma->shapes[i].type == 5) {
        CylinderPtr cylinder_map_entry_ptr = CylinderPtr(new Cylinder());
        if(!fromROSMsg(ma->shapes[i], *cylinder_map_entry_ptr)){
          continue;
        }

        cylinder_map_entry_ptr->ParamsFromShapeMsg();
        map_cylinder_.push_back(cylinder_map_entry_ptr);
//        cylinder_map_entry_ptr->dump_params("map");
        std::cout<<"MSG FROM GEOMETRY MAP\n";

        if (map_cylinder_.size() == seg_cylinder_.size()) {
          evaluate();
        }






      }
    }


  }

  void
  seg_callback(const cob_3d_mapping_msgs::ShapeArray::ConstPtr sa)
  {
    tf::StampedTransform trf_map;
    Eigen::Affine3f af = Eigen::Affine3f::Identity();


    try
    {
      tf_listener_.waitForTransform(map_frame_id_, sa->header.frame_id, sa->header.stamp, ros::Duration(2));
      tf_listener_.lookupTransform(map_frame_id_, sa->header.frame_id, sa->header.stamp, trf_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[geometry map node] : %s",ex.what());
      return;
    }
    Eigen::Affine3d ad;
    tf::TransformTFToEigen(trf_map, ad);
    af = ad.cast<float>();




    for(unsigned int i=0; i<sa->shapes.size(); i++)
    {

      ////    distinction of type
      if (sa->shapes[i].type == 0) {


        PolygonPtr polygon_map_entry_ptr = PolygonPtr(new Polygon());
        if(!fromROSMsg(sa->shapes[i], *polygon_map_entry_ptr)) {
          std::cout << "ERROR: fromROSMsg" << std::endl;
          continue;
        }
        polygon_map_entry_ptr->transform2tf(af);
      }

      if (sa->shapes[i].type == 5) {
        CylinderPtr cylinder_map_entry_ptr = CylinderPtr(new Cylinder());
        //        cylinder_map_entry_ptr->allocate();
        if(!fromROSMsg(sa->shapes[i], *cylinder_map_entry_ptr)){
          continue;
        }

        cylinder_map_entry_ptr->transform2tf(af);
        cylinder_map_entry_ptr->ParamsFromShapeMsg();

//        cylinder_map_entry_ptr->dump_params("seg");
        seg_cylinder_.push_back(cylinder_map_entry_ptr);
        std::cout<<"MSG FROM SEGMENTATION\n";


      }




    }
  }

  void
  evaluate()
  {
    ++era_;
    std::cout<<"evalating..\n";
    CylinderPtr d_c=CylinderPtr(new Cylinder);
    for (size_t i = 0; i < map_cylinder_.size(); ++i) {

        d_c->r_ = map_cylinder_[i]->r_ - seg_cylinder_[i]->r_;
//        diff_cyl_.normal = map_cylinder_[i]->normal - seg_cylinder_[i]->normal;
        d_c->sym_axis = map_cylinder_[i]->sym_axis- seg_cylinder_[i]->sym_axis;
        d_c->origin_ = map_cylinder_[i]->origin_ -seg_cylinder_[i]->origin_;
        d_c->h_min_ = map_cylinder_[i]->h_min_ - seg_cylinder_[i]->h_min_;
        d_c->h_max_ = map_cylinder_[i]->h_max_ - seg_cylinder_[i]->h_max_;
        diff_cylinder_.push_back(d_c);



    }
    dump(diff_cylinder_,"DIFF",era_);
    dump(seg_cylinder_,"SEG",era_);
    dump(map_cylinder_,"MAP",era_);

  }

  void
  dump(std::vector<CylinderPtr>& c_vec,std::string str,unsigned int& e)
  {
    std::string  e_str;
    e_str = make_str(e);
    str.append(e_str);
    for (size_t i = 0; i < c_vec.size(); ++i) {
      c_vec[i]->dump_params(str);
    }
  }





std::string make_str(unsigned int& a)
{
  std::stringstream ss;//create a stringstream
     ss << a;//add number to the stream
     return ss.str();//return a string with the contents of the stream
  }
  ros::NodeHandle n_;


protected:


  ros::Subscriber seg_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher marker_pub_;
  tf::TransformListener tf_listener_;

  std::string map_frame_id_;

  std::vector<CylinderPtr> seg_cylinder_,map_cylinder_,diff_cylinder_;

  unsigned int era_;



};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "eval_node");

  eval_node evn;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    ros::spinOnce ();
    loop_rate.sleep();
  }
}

