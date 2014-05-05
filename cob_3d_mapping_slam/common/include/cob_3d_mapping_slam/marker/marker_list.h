/*
 * marker_list.h
 *
 *  Created on: 14.11.2012
 *      Author: josh
 */

#ifndef MARKER_LIST_H_
#define MARKER_LIST_H_

#include <visualization_msgs/Marker.h>

namespace cob_3d_marker {

  class MarkerContainer;

  class MarkerList {
    int id_;

  protected:

    visualization_msgs::Marker marker_tmpl_;
    std::vector<visualization_msgs::Marker> marker_;

    static int getNewId() {
      static int id=0;
      return ++id;
    }

  public:
    MarkerList(const int id_) : id_(id_)
    {

      marker_tmpl_.header.frame_id = "/openni_rgb_optical_frame";
      marker_tmpl_.pose.position.x = 0;
      marker_tmpl_.pose.position.y = 0;
      marker_tmpl_.pose.position.z = 0;
      marker_tmpl_.pose.orientation.x = 0.0;
      marker_tmpl_.pose.orientation.y = 0.0;
      marker_tmpl_.pose.orientation.z = 0.0;
      marker_tmpl_.pose.orientation.w = 1.0;
      marker_tmpl_.action = visualization_msgs::Marker::ADD;
      marker_tmpl_.color.r = marker_tmpl_.color.g = marker_tmpl_.color.b =  marker_tmpl_.color.a = 1;
      marker_tmpl_.scale.x = marker_tmpl_.scale.y = marker_tmpl_.scale.z = 0.01;
      marker_tmpl_.id = getNewId();
    }

    int getId() const {return id_;}

    friend class MarkerContainer;
  };

  class MarkerList_Line : public MarkerList {
  public:

    MarkerList_Line(const int id):MarkerList(id) {
      marker_tmpl_.type = visualization_msgs::Marker::LINE_LIST;
      marker_.push_back(marker_tmpl_);
    }

    void addLine(const Eigen::Vector3f &va, const Eigen::Vector3f &vb, const float r=1, const float g=1, const float b=1, const float a=1) {
      geometry_msgs::Point line_p;
      ::std_msgs::ColorRGBA col;

      col.r = r;
      col.g = g;
      col.b = b;
      col.a = a;
      line_p.x = va(0);
      line_p.y = va(1);
      line_p.z = va(2);
      marker_.back().points.push_back(line_p);
      marker_.back().colors.push_back(col);

      line_p.x = vb(0);
      line_p.y = vb(1);
      line_p.z = vb(2);
      marker_.back().points.push_back(line_p);
      marker_.back().colors.push_back(col);
    }

  };

  class MarkerList_Triangles : public MarkerList {
  public:

    MarkerList_Triangles(const int id):MarkerList(id) {
      marker_tmpl_.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker_tmpl_.scale.x=marker_tmpl_.scale.y=marker_tmpl_.scale.z=1;
      marker_.push_back(marker_tmpl_);
    }

    void addTriangle(const Eigen::Vector3f &va, const Eigen::Vector3f &vb, const Eigen::Vector3f &vc, const float r=1, const float g=1, const float b=1, const float a=1) {
      geometry_msgs::Point line_p;
      ::std_msgs::ColorRGBA col;

      col.r = r;
      col.g = g;
      col.b = b;
      col.a = a;
      line_p.x = va(0);
      line_p.y = va(1);
      line_p.z = va(2);
      marker_.back().points.push_back(line_p);
      marker_.back().colors.push_back(col);

      line_p.x = vb(0);
      line_p.y = vb(1);
      line_p.z = vb(2);
      marker_.back().points.push_back(line_p);
      marker_.back().colors.push_back(col);

      line_p.x = vc(0);
      line_p.y = vc(1);
      line_p.z = vc(2);
      marker_.back().points.push_back(line_p);
      marker_.back().colors.push_back(col);
    }

  };

  class MarkerList_Arrow : public MarkerList {
  public:

    MarkerList_Arrow(const int id):MarkerList(id)
    {
      marker_tmpl_.type = visualization_msgs::Marker::ARROW;
    }

    void addArrow(const Eigen::Vector3f &va, const Eigen::Vector3f &vb, const float r=1, const float g=1, const float b=1, const float a=1) {
      visualization_msgs::Marker m = marker_tmpl_;

      geometry_msgs::Point line_p;
      ::std_msgs::ColorRGBA col;

      m.scale.x = 0.025;
      m.scale.y = 0.05;
      m.scale.z = 0.1;

      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = a;

      line_p.x = va(0);
      line_p.y = va(1);
      line_p.z = va(2);
      m.points.push_back(line_p);

      line_p.x = vb(0);
      line_p.y = vb(1);
      line_p.z = vb(2);
      m.points.push_back(line_p);

      m.id = getNewId();

      marker_.push_back(m);
    }

  };

  class MarkerList_Text : public MarkerList {
  public:

    MarkerList_Text(const int id):MarkerList(id)
    {
      marker_tmpl_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    }

    void addText(const Eigen::Vector3f &pos, const std::string &text, const float size=0.1f, const float r=1, const float g=1, const float b=1, const float a=1) {
      visualization_msgs::Marker m = marker_tmpl_;

      m.pose.position.x = pos(0);
      m.pose.position.y = pos(1);
      m.pose.position.z = pos(2);

      m.text = text;

      m.scale.z = size;

      m.id = getNewId();

      marker_.push_back(m);
    }

  };

}



#endif /* MARKER_LIST_H_ */
