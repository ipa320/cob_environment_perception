/*
 * form.h
 *
 *  Created on: 18.07.2012
 *      Author: josh
 */

#ifndef FORM_H_
#define FORM_H_

#include "polygon_merger.h"

namespace Slam_CurvedPolygon
{

  class Outline
  {

    static void debug_svg_text(FILE *fp, float x, float y, float e, const char *color) {
      char buffer[1024];
      sprintf(buffer,"<text x=\"%f\" y=\"%f\"  style=\"stroke:%s; stroke-width:2px;\">%.2f</text>",x,y,color,e);
      fputs(buffer,fp);
    }
    static void debug_svg_line(FILE *fp, float x, float y, float x2, float y2, float e, const char *color="black") {
      char buffer[1024];
      sprintf(buffer,"<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:%s; stroke-width:2px;\" />",x,y,x2,y2,color);
      if(pcl_isfinite(e))
        debug_svg_text(fp,(x+x2)/2,(y+y2)/2,e,color);
      fputs(buffer,fp);
    }
    static void debug_svg_line(FILE *fp, float x, float y, float x2, float y2,  const char *color="black") {
      char buffer[1024];
      sprintf(buffer,"<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:%s; stroke-width:2px;\" />",x,y,x2,y2,color);
      fputs(buffer,fp);
    }

  public:
    std::vector<Eigen::Vector3f> segments_;
    float weight_;
    bool open_;

    Outline(): weight_(1.f), open_(false)
    {}

    void operator+=(const Eigen::Vector3f &v) {
      segments_.push_back(v);
      if(!pcl_isfinite(segments_.back()(2)) ) {
        std::cout<<"v is not finite\n"<<segments_.back()<<std::endl;
        ROS_ERROR("v is not finite");
        segments_.back()(2) = 0.f;
      }
    }

    struct OUTPUTSVG {
      FILE *fp;
      std::string fn;

      OUTPUTSVG(const char *fn):fn(fn) {
        fp=fopen((std::string("/tmp/")+fn).c_str(),"w");
        fputs("<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"no\" ?>",fp);
        fputs("<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 20010904//EN\" \"http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd\">",fp);
        fputs("<svg width=\"1640\" height=\"1480\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\">",fp);
      }

      ~OUTPUTSVG()
      {
        fputs("</svg>",fp);
        fclose(fp);
      }
    };

    boost::shared_ptr<OUTPUTSVG> debug_svg(const char *fn, const float f=100, const float o=300, const char *color="black") const {
      boost::shared_ptr<OUTPUTSVG> out(new OUTPUTSVG(fn));
      return debug_svg(out,f,o, color);
    }

    boost::shared_ptr<OUTPUTSVG> debug_svg(boost::shared_ptr<OUTPUTSVG> out, const float f=100, const float o=300, const char *color="black") const {
      FILE *fp2=fopen((std::string("/tmp/data_")+out->fn+".txt").c_str(),"a");

      fprintf(fp2,"\n\n");

      for(size_t i=0; i<segments_.size(); i++)
      {
        Eigen::Vector2f p=segments_[(i+1)%segments_.size()].head<2>();
        Eigen::Vector2f p2=segments_[i].head<2>();
        Eigen::Vector2f d=p2-p;
        debug_svg_line(out->fp, segments_[i](0)*f+o, segments_[i](1)*f+o,
                       segments_[(i+1)%segments_.size()](0)*f+o, segments_[(i+1)%segments_.size()](1)*f+o
                       ,segments_[(i+1)%segments_.size()](2)/weight_
                       ,color
        );
        debug_svg_line(out->fp, p(0)*f+o,p(1)*f+o,
                       (p(0)+0.1*d(0)+0.1*d(1))*f+o,(p(1)+0.1*d(1)-0.1*d(0))*f+o,color);
        debug_svg_line(out->fp, p(0)*f+o,p(1)*f+o,
                       (p(0)+0.1*d(0)-0.1*d(1))*f+o,(p(1)+0.1*d(1)+0.1*d(0))*f+o,color);
        debug_svg_text(out->fp,p2(0)*f+o,p2(1)*f+o,i,color);

        fprintf(fp2,"ADD_POINT(OUTLINE, %f,%f, \t%f);\n",segments_[i](0),segments_[i](1),segments_[i](2));
      }

      fclose(fp2);
      return out;
    }

    struct COR
    {
      bool other;
      size_t ind;
      Eigen::Vector3f p;
    };

    template<typename T>
    static size_t nextLine(const std::vector<T> &segments_, const Eigen::Vector2f &p, const Eigen::Vector2f &n, T &r) {
      size_t ind = (size_t)-1;
      Eigen::Matrix2f M;
      M.col(0) = n;
      float mi = std::numeric_limits<float>::max();
      for(size_t i=0; i<segments_.size(); i++)
      {
        Eigen::Vector2f p3 = segments_[i].head(2);
        Eigen::Vector2f p4 = segments_[(i+1)%segments_.size()].head(2);

        M.col(1) = p3-p4;

        Eigen::Vector2f m = (M.inverse()*(p3-p));
        float d;

        if(m(1)>=0.f && m(1)<=1.f)
          d = (m(0)*n).squaredNorm();
        if(m(1)<0)
          d = (p3-p).squaredNorm();
        else
          d = (p4-p).squaredNorm();

        //        std::cout<<" "<<i<<"  "<<m(0)<<"  "<<m(1)<<"\n";

        if(d<mi)
        {
          mi = d;
          ind = (i+1)%segments_.size();
          if(m(1)>0.f && m(1)<1.f) {
            r.head(2) = m(0)*n+p;
            r(2) = segments_[ind](2);
            r(3) = segments_[ind](3);
          }
          else if(m(1)<=0) {
            r.head(2) = p3;
            r(2) = std::max(segments_[ind](2),segments_[i](2));
            if(segments_[ind](2)>segments_[i](2))
              r(3) = segments_[ind](3);
            else
              r(3) = segments_[i](3);
            ind = i;
          }
          else {
            r.head(2) = p4;
            r(2) = std::max(segments_[ind](2),segments_[(ind+1)%segments_.size()](2));
            if(segments_[ind](2)>segments_[(ind+1)%segments_.size()](2))
              r(3) = segments_[ind](3);
            else
              r(3) = segments_[(ind+1)%segments_.size()](3);
          }
        }
        //m(0) =
        //	  (
        //  (p1(0)*p2(1)-p1(0)*p2(0))*(p3(0)-p4(0)) -
        //(p1(0)-p2(0))*(p3(0)*p4(1);
      }
      return ind;
    }

    size_t _nextPoint(const Eigen::Vector2f &p) const {
      size_t ind = (size_t)-1;
      float mi = std::numeric_limits<float>::max();
      for(size_t i=0; i<segments_.size(); i++)
      {
        const float d= (segments_[i].head<2>()-p).squaredNorm();

        if(d<mi)
        {
          mi = d;
          ind = i;
        }
      }
      return ind;
    }

    size_t _nextPoint2(const Eigen::Vector2f &p, Eigen::Vector2f &r) const {
      size_t ind = (size_t)-1;
      float mi = std::numeric_limits<float>::max();
      for(size_t i=0; i<segments_.size(); i++)
      {
        Eigen::Vector2f p1 = segments_[i].head<2>();
        Eigen::Vector2f p2 = segments_[(i+1)%segments_.size()].head<2>();

        Eigen::Vector2f d1 = (p-p1);
        Eigen::Vector2f d2 = p2-p1;
        float f = d1.dot(d2)/d2.squaredNorm();
        float d;
        Eigen::Vector2f n;

        size_t tmp = i+1;
        if(f>=1){
          d = (p-p2).norm();
          n = p2-p;
        }
        else if(f<=0) {
          d = (p-p1).norm();
          n = p1-p;
          tmp--;
        }
        else {
          d = (d1(0)*d2(1)-d1(1)*d2(0))/(d2).norm();
          n(0) = -(p2(1)-p1(1));
          n(1) = (p2(0)-p1(0));
        }

        if(d<mi)
        {
          mi = d;
          ind = tmp;
          r = n+p;
        }
      }
      return ind;
    }

    operator std::vector<Eigen::Vector4f>() const {
      std::vector<Eigen::Vector4f> r;
      for(size_t i=0; i<size(); i++)
      {
        Eigen::Vector4f v;
        v.head<3>() = (*this)[i];
        v(3) = weight_;
        r.push_back(v);
      }
      return r;
    }

    static size_t _hasIntersection(const std::vector<Eigen::Vector4f> &segments_, const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const bool open=false) {
      Eigen::Matrix2f M;
      M.col(0) = p2-p1;
      for(size_t i=0; i<segments_.size()-(open?1:0); i++)
      {
        Eigen::Vector2f p3 = segments_[i].head(2);
        Eigen::Vector2f p4 = segments_[(i+1)%segments_.size()].head(2);
        M.col(1) = p3-p4;

        Eigen::Vector2f m = (M.inverse()*(p3-p1));

        if(m(0)>0.001&&m(0)<0.99999 && m(1)>0.001&&m(1)<0.99999) {
//          ROS_ERROR("intersection %f %f",m(0),m(1));
//          std::cerr<<m(0)*M.col(0)+p1<<"\n";
//          std::cerr<<m(1)*M.col(1)+p3<<"\n";
          return i+1;
        }
      }
      return 0;
    }

    static size_t _nextPoint3(const std::vector<Eigen::Vector4f> &segments_, const Eigen::Vector2f &p, Eigen::Vector4f &r, const bool open=false) {
      size_t ind = (size_t)-1;
      float mi = std::numeric_limits<float>::max();
      for(size_t i=0; i<segments_.size()-(open?1:0); i++)
      {
        Eigen::Vector2f p1 = segments_[i].head<2>();
        Eigen::Vector2f p2 = segments_[(i+1)%segments_.size()].head<2>();

        Eigen::Vector2f d1 = (p-p1);
        Eigen::Vector2f d2 = p2-p1;
        float f = d1.dot(d2)/(d2.squaredNorm());
        float d;
        Eigen::Vector2f n;

        size_t tmp = i+1;
        if(f>0&&f<1) {
          d = (d1(0)*d2(1)-d1(1)*d2(0))/(d2).norm();
          n = f*d2 + p1;
        }
        else if(f>=1){
          d = (p-p2).norm();
          n = p2;
        }
        else {
          d = (p-p1).norm();
          n = p1;
          tmp--;
        }

        if(d<mi)
        {
          //check if line intersects itself
          if(_hasIntersection(segments_,p,n))
            continue;

          mi = d;
          ind = tmp;
          r.head<2>() = n;
          if(f>0&&f<1) {
            r(2) = segments_[(i+1)%segments_.size()](2);
            r(3) = segments_[(i+1)%segments_.size()](3);
          }
          else if(f>=1){
            if( segments_[(i+1)%segments_.size()](2)/segments_[(i+1)%segments_.size()](3) >
            segments_[(i+2)%segments_.size()](2)/segments_[(i+2)%segments_.size()](3)) {
              r(2) = segments_[(i+1)%segments_.size()](2);
              r(3) = segments_[(i+1)%segments_.size()](3);
            } else {
              r(2) = segments_[(i+2)%segments_.size()](2);
              r(3) = segments_[(i+2)%segments_.size()](3); }
          }
          else{
            if( segments_[(i+1)%segments_.size()](2)/segments_[(i+1)%segments_.size()](3) >
            segments_[i](2)/segments_[i](3)) {
              r(2) = segments_[(i+1)%segments_.size()](2);
              r(3) = segments_[(i+1)%segments_.size()](3);
            } else {
              r(2) = segments_[i](2);
              r(3) = segments_[i](3); }
          }
        }
      }
      return ind;
    }

    static void run2(const std::vector<Eigen::Vector4f> &segments_, const std::vector<Eigen::Vector4f> &o, const size_t begin, const size_t end, Outline &res, const bool proc=true) {
      size_t last_ind, start_ind;
      bool start = true;
      Eigen::Vector3f r2;

      //      ROS_ERROR("%d %d",begin, end);
      for(size_t j=begin; j<(end>begin?end:end+segments_.size()); j++)
      {
        size_t i = j%segments_.size();

        Eigen::Vector2f n1,n2;
        Eigen::Vector4f r;
        n1=segments_[i].head<2>()-segments_[(i-1+segments_.size())%segments_.size()].head<2>();
        n1.normalize();
        n2=segments_[(i+1)%segments_.size()].head<2>()-segments_[i].head<2>();
        n2.normalize();
        n2+=n1;
        n1(0) = -n2(1);
        n1(1) =  n2(0);
        size_t ind = nextLine(o, segments_[i].head<2>(), n1, r);
        if(ind!=(size_t)-1) {
          //          ROS_ERROR("%d",ind);
          if(!start && ind!=last_ind && proc) {
            run2(o, segments_, last_ind, ind, res, false);
            //            if(ind<last_ind) return;
            //            ROS_ASSERT(ind>=last_ind);
          }
          else if(start) {
            start = false;
            start_ind = ind;
          }
          last_ind = ind;

          const float p1 = std::max(segments_[i](2), segments_[(i+1)%segments_.size()](2))+0.001f;
          float weight;
          if(segments_[i](2)>segments_[(i+1)%segments_.size()](2))
            weight = segments_[i](3);
          else
            weight = segments_[(i+1)%segments_.size()](3);
          const float p2 = r(2)+0.001f;

          const float ramp = std::max((p1-0.001f)/weight,r(2)/r(3));
          const bool out = n1.dot(r.head<2>()-segments_[i].head<2>())<0;
          const float w1 = (!out?1.f:ramp);
          const float w2 = (out?1.f:ramp);
          r2.head<2>() = w1*p2/(w2*p1+w1*p2)*(r.head<2>()-segments_[i].head<2>()) + segments_[i].head<2>();
          r2(2) = (p1+p2-0.002f)/(weight+r(3));

          res += r2;

          //          static int ctr=0;
          //          char buf[128];
          //          sprintf(buf,"%d.svg",ctr++);
          //          res.debug_svg(buf);
        }
        //        else
        //          ROS_ASSERT(0);
      }
      if(start_ind!=last_ind&&proc)
        run2(o, segments_, last_ind, start_ind, res, false);
    }

#if 0
    void run(const Outline &o, const size_t begin, const size_t end, Outline &res, const bool proc=true) const {
      size_t last_ind, start_ind;
      bool start = true;
      Eigen::Vector3f r2;

      ROS_ERROR("%d %d",begin, end);
      for(size_t j=begin; j<(end>begin?end:end+segments_.size()); j++)
      {
        size_t i = j%segments_.size();

        Eigen::Vector2f n1,n2;
        Eigen::Vector3f r;
        n1=segments_[i].head<2>()-segments_[(i-1+segments_.size())%segments_.size()].head<2>();
        n1.normalize();
        n2=segments_[(i+1)%segments_.size()].head<2>()-segments_[i].head<2>();
        n2.normalize();
        n2+=n1;
        n1(0) = -n2(1);
        n1(1) =  n2(0);
        size_t ind = o.nextLine(segments_[i].head<2>(), n1, r);
        if(ind!=(size_t)-1) {
          ROS_ERROR("%d",ind);
          if(!start && ind!=last_ind && proc) {
            o.run(*this, last_ind, ind, res, false);
            //            if(ind<last_ind) return;
            //            ROS_ASSERT(ind>=last_ind);
          }
          else if(start) {
            start = false;
            start_ind = ind;
          }
          last_ind = ind;

          const float p1 = std::max(segments_[i](2), segments_[(i+1)%segments_.size()](2))+0.001f;
          const float p2 = r(2)+0.001f;

          const float ramp = std::max((p1-0.001f)/weight_,(p2-0.001f)/o.weight_);
          const bool out = n1.dot(r.head<2>()-segments_[i].head<2>())<0;
          const float w1 = (!out?1.f:ramp);
          const float w2 = (out?1.f:ramp);
          r2.head<2>() = w1*p2/(w2*p1+w1*p2)*(r.head<2>()-segments_[i].head<2>()) + segments_[i].head<2>();
          r2(2) = (p1+p2-0.002f)/(o.weight_+weight_);

          res += r2;

          //          static int ctr=0;
          //          char buf[128];
          //          sprintf(buf,"%d.svg",ctr++);
          //          res.debug_svg(buf);
        }
        //        else
        //          ROS_ASSERT(0);
      }
      if(start_ind!=last_ind&&proc)
        o.run(*this, last_ind, start_ind, res, false);
    }
#endif

#if 0
    void run(const Outline &o, const size_t begin, const size_t end, Outline &res, const bool proc=true) const {
      size_t last_ind, start_ind;
      bool start = true;
      Eigen::Vector3f r2;

      ROS_ERROR("%d %d",begin, end);
      for(size_t j=begin; j<(end>begin?end:end+segments_.size()); j++)
      {
        size_t i = j%segments_.size();

        size_t ind = o._nextPoint(segments_[i].head<2>());
        if(ind!=(size_t)-1) {
          ROS_ERROR("%d",ind);
          if(!start && ind!=last_ind && proc) {
            o.run(*this, last_ind+1, ind, res, false);
            //            if(ind<last_ind) return;
            //            ROS_ASSERT(ind>=last_ind);
          }
          else if(start) {
            start = false;
            start_ind = ind;
          }
          last_ind = ind;

          Eigen::Vector2f n = segments_[i].head<2>()-segments_[(i+1)%segments_.size()].head<2>();
          std::swap(n(0),n(1));
          n(0) = -n(0);

          const float p1 = std::max(segments_[i](2), segments_[(i+1)%segments_.size()](2))+0.001f;
          const float p2 = std::max(o.segments_[ind](2), o.segments_[(ind+1)%segments_.size()](2))+0.001f;

          const float ramp = std::max((p1-0.001f)/weight_,(p2-0.001f)/o.weight_);
          const bool out = n.dot(o.segments_[ind].head<2>()-segments_[i].head<2>())<0;
          const float w1 = (!out?1.f:ramp);
          const float w2 = (out?1.f:ramp);
          r2.head<2>() = w1*p2/(w2*p1+w1*p2)*(o.segments_[ind].head<2>()-segments_[i].head<2>()) + segments_[i].head<2>();
          r2(2) = (p1+p2-0.002f)/(o.weight_+weight_);

          res += r2;

          //          static int ctr=0;
          //          char buf[128];
          //          sprintf(buf,"%d.svg",ctr++);
          //          res.debug_svg(buf);
        }
        //        else
        //          ROS_ASSERT(0);
      }
      if(start_ind!=last_ind&&proc)
        o.run(*this, last_ind+1, start_ind, res, false);
    }
#endif

#if 0
    void run(const Outline &o, const size_t begin, const size_t end, Outline &res, const bool proc=true) const {
      size_t last_ind, start_ind;
      bool start = true;
      Eigen::Vector3f r2;

      ROS_ERROR("%d %d",begin, end);
      for(size_t j=begin; j<(end>begin?end:end+segments_.size()); j++)
      {
        size_t i = j%segments_.size();

        size_t ind = o._nextPoint2(segments_[i].head<2>());
        if(ind!=(size_t)-1) {
          ROS_ERROR("%d",ind);
          if(!start && ind!=last_ind && proc) {
            o.run(*this, last_ind+1, ind, res, false);
            //            if(ind<last_ind) return;
            //            ROS_ASSERT(ind>=last_ind);
          }
          else if(start) {
            start = false;
            start_ind = ind;
          }
          last_ind = ind;

          Eigen::Vector2f n = segments_[i].head<2>()-segments_[(i+1)%segments_.size()].head<2>();
          std::swap(n(0),n(1));
          n(0) = -n(0);

          const float p1 = std::max(segments_[i](2), segments_[(i+1)%segments_.size()](2))+0.001f;
          const float p2 = std::max(o.segments_[ind](2), o.segments_[(ind+1)%segments_.size()](2))+0.001f;

          const float ramp = std::max((p1-0.001f)/weight_,(p2-0.001f)/o.weight_);
          const bool out = n.dot(o.segments_[ind].head<2>()-segments_[i].head<2>())<0;
          const float w1 = (!out?1.f:ramp);
          const float w2 = (out?1.f:ramp);
          r2.head<2>() = w1*p2/(w2*p1+w1*p2)*(o.segments_[ind].head<2>()-segments_[i].head<2>()) + segments_[i].head<2>();
          r2(2) = (p1+p2-0.002f)/(o.weight_+weight_);

          res += r2;

          //          static int ctr=0;
          //          char buf[128];
          //          sprintf(buf,"%d.svg",ctr++);
          //          res.debug_svg(buf);
        }
        //        else
        //          ROS_ASSERT(0);
      }
      if(start_ind!=last_ind&&proc)
        o.run(*this, last_ind+1, start_ind, res, false);
    }
#endif

    bool isPointIn(const Eigen::Vector2f &pt) const {
      bool c=false;
      size_t j=segments_.size()-1;
      for(size_t i=0; i<segments_.size(); i++)
      {
        if( (segments_[i](1)>pt(1))!=(segments_[j](1)>pt(1)) &&
            (pt(0)<(segments_[j](0)-segments_[i](0))*(pt(1)-segments_[i](1))/(segments_[j](1)-segments_[i](1))+segments_[i](0))
        )
          c=!c;
        j=i;
      }
      return c;
    }

    bool check() const {
      bool ASS=true;

      for(size_t i=0; i<size(); i++)
      {
        size_t id = _hasIntersection(*this, (*this)[i].head<2>(), (*this)[(i+1)%size()].head<2>());
        if(id && id-1!=i) {
          ROS_ERROR("err at %d (%d)",i,id-1);
          ASS=false;
          break;
        }
      }

      if(!ASS) {
        char buffer[128];
        static int n=0;
        sprintf(buffer, "error%d.svg",n++);
        debug_svg(buffer,300,600);
      }

      return ASS;
    }

    void run(const Outline &o, const size_t begin, const size_t end, Outline &res, const bool proc=true) const {
      std::vector<Eigen::Vector4f> inner, outer;

      std::vector<Eigen::Vector4f> l1, l2;
      int type = -1;

      //start with point inside
      bool found=false, found1;
      size_t i=0;
      for(; i<segments_.size(); i++)
      {
        if(o.isPointIn(segments_[i].head<2>())) {
          found=true;
          break;
        }
      }
      found1 = found;

      if(found==false)
      {
        for(size_t i=0; i<o.segments_.size(); i++)
        {
          if(isPointIn(o.segments_[i].head<2>())) {
            found=true;
            break;
          }
        }
      }

      if(found)
      {

        bool other=false;
        size_t j=0, last_j=0, start_j=0;
        for(size_t kk=0; kk<segments_.size(); kk++)
        {
          i%=segments_.size();

          Eigen::Matrix2f M;
          Eigen::Vector2f p1 = segments_[i].head<2>();
          Eigen::Vector2f p2 = segments_[(i+1)%segments_.size()].head<2>();

          Eigen::Vector4f p14;
          p14.head<3>() = segments_[i];
          p14(3) = weight_;
          l1.push_back(p14);

          M.col(0) = p2-p1;

          if(type==-1)
          {
            float mi=10;
            size_t mm=0;
            for(size_t k=0; k<o.segments_.size(); k++)
            {
              j%=o.segments_.size();
              Eigen::Vector2f p3 = o.segments_[j].head<2>();
              Eigen::Vector2f p4 = o.segments_[(j+1)%o.segments_.size()].head<2>();

              M.col(1) = p3-p4;

              Eigen::Vector2f m = (M.inverse()*(p3-p1));

              if(m(0)>=0.f && m(0)<1.f && m(1)>=0.f && m(1)<1.f)
              {
                if(m(0)<mi) {
                  mi=m(0);
                  mm=j;
                }
              }

              ++j;
            }

            if(mi<=1)
              j=mm;
          }
          for(size_t k=0; k<o.segments_.size(); k++)
          {
            j%=o.segments_.size();
            Eigen::Vector2f p3 = o.segments_[j].head<2>();
            Eigen::Vector2f p4 = o.segments_[(j+1)%o.segments_.size()].head<2>();

            M.col(1) = p3-p4;

            Eigen::Vector2f m = (M.inverse()*(p3-p1));

            if(m(0)>0.f && m(0)<1.f && m(1)>0.f && m(1)<1.f)
            {
              Eigen::Vector3f v1,v2;
              v1.head<2>() = M.col(0);
              v2.head<2>() = M.col(1);
              v1(2)=v2(2)=0;
              const bool out = v1.cross(v2)(2)<=0;

//              ROS_INFO("INTERSECTION %f %f %d",m(0),v1.cross(v2)(2),type);

              if(type!=-1) {
                size_t end = j;//(j+(out?0:1))%o.segments_.size();
                //ROS_ERROR("j %d %d %d",last_j,end,j);
                if(last_j!=end)
                  //              if((last_j+1)%o.segments_.size()==end)
                  //              {
                  //                for(size_t t=0; t<o.segments_.size(); t++) {
                  //                  Eigen::Vector4f p34;
                  //                  p34.head<3>() = o.segments_[t];
                  //                  p34(3) = o.weight_;
                  //                  l2.push_back(p34);
                  //                }
                  //              }
                  //              else
                  for(size_t t=(last_j+1)%o.segments_.size(); true; t=(t+1)%o.segments_.size()) {
                    Eigen::Vector4f p34;
                    p34.head<3>() = o.segments_[t];
                    p34(3) = o.weight_;
                    l2.push_back(p34);

                    if(t==end)
                      break;
                  }
              }
              else
                start_j = j;
              //intersection
              Eigen::Vector4f p;
              p.head<2>() = m(0)*M.col(0) + p1;

              p(2) = segments_[i](2);
              p(3) = weight_;
              l1.push_back(p);
              p(2) = o.segments_[j](2);
              p(3) = o.weight_;
              l2.push_back(p);

              //add to outline, inline
              std::vector<Eigen::Vector4f> *pl1=&l1, *pl2=&l2;
              if(out) { //outer
                std::swap(pl1,pl2);
                type=0;
              }
              else
                type=1;

              //ROS_ERROR("s %d %d",pl1->size(),pl2->size());

              for(size_t l=0; l<pl1->size(); l++)
                inner.push_back( (*pl1)[l] );
              for(size_t l=0; l<pl2->size(); l++)
                outer.push_back( (*pl2)[l] );

              //            res += p.head<3>();

              //remove them
              l1.clear();
              l2.clear();

              last_j=j;

              if(last_j!=start_j)
                other=true;
            }

            ++j;
          }
          j=last_j;

          ++i;
        }

        std::vector<Eigen::Vector4f> *pl1=&l1, *pl2=&l2;
        if(type==1) {
          std::swap(pl1,pl2);
        }
        if(type==1 || type==0) {
          //ROS_ERROR("j %d %d",last_j,start_j);
          if(last_j!=start_j || !other)
          for(size_t t=(last_j+1)%o.segments_.size(); true; t=(t+1)%o.segments_.size()) {
            Eigen::Vector4f p34;
            p34.head<3>() = o.segments_[t];
            p34(3) = o.weight_;
            l2.push_back(p34);

            if(t==start_j)
              break;
          }
//          for(size_t t=(last_j+1)%o.segments_.size(); t!=start_j; t=(t+1)%o.segments_.size()) {
//            Eigen::Vector4f p34;
//            p34.head<3>() = o.segments_[t];
//            p34(3) = o.weight_;
//            l2.push_back(p34);
//          }

          //ROS_ERROR("s %d %d",pl1->size(),pl2->size());

          for(size_t l=0; l<pl1->size(); l++)
            inner.push_back( (*pl1)[l] );
          for(size_t l=0; l<pl2->size(); l++)
            outer.push_back( (*pl2)[l] );
        }

      }

      if(type==-1)
      {
        Eigen::Vector4f v;
        for(size_t l=0; l<segments_.size(); l++) {
          v.head<3>() = segments_[l];
          v(3) = weight_;
          outer.push_back( v );
        }
        for(size_t l=0; l<o.segments_.size(); l++) {
          v.head<3>() = o.segments_[l];
          v(3) = o.weight_;
          inner.push_back( v );
        }

        if(found1)
          std::swap(outer,inner);
      }

      //ROS_ERROR("size %d %d %d %d",outer.size(),inner.size(),size(),o.size());

      if(!(outer.size()<=2*(size()+o.size())) || !(inner.size()<=2*(size()+o.size())))
      {
        o.debug_svg(debug_svg("error.svg"),100,300,"red");

        Outline oi, oo;
        for(size_t i=0; i<inner.size(); i++)
        {
          Eigen::Vector3f r2;
          r2.head<2>() = inner[i].head<2>();
          r2(2) = inner[i](2)/inner[i](3);
          oi += r2;
        }
        for(size_t i=0; i<outer.size(); i++)
        {
          Eigen::Vector3f r2;
          r2.head<2>() = outer[i].head<2>();
          r2(2) = outer[i](2)/outer[i](3);
          oo += r2;
        }
        oo.debug_svg(oi.debug_svg("error_io.svg"),100,300,"red");
      }
//      ROS_ASSERT(outer.size()<=2*(size()+o.size()));
//      ROS_ASSERT(inner.size()<=2*(size()+o.size()));

      //run through outer
      //run2(inner, outer, 0, inner.size(), res);
#if 1
      bool ASS=false;

      for(size_t i=0; i<outer.size(); i++)
      {
        Eigen::Vector3f r2;
        Eigen::Vector4f r;

        size_t ind = _nextPoint3(inner,outer[i].head<2>(),r);

        if(0&&ind<inner.size())
        {

          const float p1 = outer[i](2)+0.001f;
          float weight = outer[i](3);
          const float p2 = r(2)+0.001f;

          const float ramp = std::max((p1-0.001f)/weight,r(2)/r(3));
          const float w1 = ramp;
          const float w2 = 1.f;
          ROS_ERROR("%f %f",ramp,w1*p2/(w2*p1+w1*p2));

          r2.head<2>() = w1*p2/(w2*p1+w1*p2)*(r.head<2>()-outer[i].head<2>()) + outer[i].head<2>();
          r2(2) = (w2*p1+w1*p2-0.002f)/(w2*weight+w1*r(3));
        }
        else
        {
          r2.head<2>() = outer[i].head<2>();
          r2(2) = outer[i](2)/outer[i](3);
        }

        if(res.size() && _hasIntersection(res,res.segments_.back().head<2>(),r2.head<2>(), true)) {
          o.debug_svg(debug_svg("error.svg",300,600),300,600,"red");

          Outline oi, oo;
          for(size_t i=0; i<inner.size(); i++)
          {
            Eigen::Vector3f r2;
            r2.head<2>() = inner[i].head<2>();
            r2(2) = inner[i](2)/inner[i](3);
            oi += r2;
          }
          for(size_t i=0; i<outer.size(); i++)
          {
            Eigen::Vector3f r2;
            r2.head<2>() = outer[i].head<2>();
            r2(2) = outer[i](2)/outer[i](3);
            oo += r2;
          }
          oo.debug_svg(oi.debug_svg("error_io.svg",300,600),300,600,"red");
          ASS=true;
          break;
        }

        res += r2;
      }

      //            for(size_t i=0; i<inner.size(); i++)
      //            {
      //              Eigen::Vector3f r2;
      //              r2.head<2>() = inner[i].head<2>();
      //              r2(2) = inner[i](2)/inner[i](3);
      //              res += r2;
      //            }

      if(ASS) {
        res.debug_svg(o.debug_svg(debug_svg("error.svg",300,600),300,600,"red"),300,600,"green");
      }
      //ROS_ASSERT(!ASS);
#endif
      if(!res.check()) {  //DEBUG
        res.debug_svg(o.debug_svg(debug_svg("error.svg",300,600),300,600,"red"),300,600,"green");

        Outline oi, oo;
        for(size_t i=0; i<inner.size(); i++)
        {
          Eigen::Vector3f r2;
          r2.head<2>() = inner[i].head<2>();
          r2(2) = inner[i](2)/inner[i](3);
          oi += r2;
        }
        for(size_t i=0; i<outer.size(); i++)
        {
          Eigen::Vector3f r2;
          r2.head<2>() = outer[i].head<2>();
          r2(2) = outer[i](2)/outer[i](3);
          oo += r2;
        }
        oo.debug_svg(oi.debug_svg("error_io.svg"),100,300,"red");
        //ROS_ASSERT(0);
      }
    }

    Outline operator+(const Outline &o) const
    {
      //      ROS_ERROR("start %d %d",segments_.size(),o.segments_.size());

      Outline res;
      run(o, 0, segments_.size(), res);

      //      ROS_ERROR("%d %d %d",res.size(),segments_.size(),o.segments_.size());
      //ROS_ASSERT(res.size() == (segments_.size()+o.segments_.size()) );

      return res;
    }

#if 0
    Outline operator+(const Outline &o) const
    {

      std::vector<COR> cors;

      for(size_t i=0; i<o.segments_.size(); i++)
      {
        COR c;
        c.other = true;
        c.ind = i;
        cors.push_back(c);
      }

      for(size_t i=0; i<segments_.size(); i++)
      {
        bool out = false;
        float mi = std::numeric_limits<float>::max(), mi2 = std::numeric_limits<float>::max();
        Eigen::Vector2f nmi = Eigen::Vector2f::Zero();
        size_t ind=cors.size();
        for(size_t j=0; j<cors.size(); j++)
        {
          if(!cors[j].other) continue;

          Eigen::Vector2f n;
          Eigen::Vector2f p1,p2, x, x2, d1,d2;
          x = segments_[i].head<2>();
          x2 = segments_[(i+1)%segments_.size()].head<2>();
          p1 = o.segments_[cors[j].ind].head<2>();
          p2 = o.segments_[(cors[j].ind+1)%o.segments_.size()].head<2>();

          d1 = (x-p1);
          d2 = p2-p1;
          float f = d1.dot(d2)/d2.squaredNorm();
          float d;
          if(f>1){
            d = (x-p2).norm();
            n = p2-x;
          }
          else if(f<0) {
            d = d1.norm();
            n = p1-x;
          }
          else {
            d = (d1(0)*d2(1)-d1(1)*d2(0))/(d2).norm();
            n(0) = -(p2(1)-p1(1));
            n(1) = (p2(0)-p1(0));
          }

          const float dd = d / std::max(0.0001f, (x-x2).dot(p1-p2)/((x-x2).norm()*(p1-p2).norm()) );

          //          ROS_INFO("%f %f %f",f,d,mi);

          if(std::abs(dd)<std::abs(mi2))
          {
            nmi = n;
            mi = d;
            mi2 = dd;
            ind = j;
            out = d*(-d2(1)*n(0)+d2(0)*n(1))>-0.001f;
            //            ROS_INFO("out2 %f",d*(-d2(1)*n(0)+d2(0)*n(1)));
          }
        }

        if(cors.size()==ind)
          return Outline();

        ROS_ASSERT(cors.size()!=ind);

        //        ROS_INFO("-------------");

        for(size_t k=0; k<cors.size(); k++)
        {
          ind = (ind+1)%cors.size();
          //          ROS_INFO("cmp %d %d", (int)std::abs((int)cors[ind].ind-(int)i), (int)std::abs((int)cors[ind].ind+segments_.size()-(int)i));
          if(cors[ind].other || std::abs((int)cors[ind].ind-(int)i)>std::abs((int)cors[ind].ind+segments_.size()-(int)i))
            break;
        }

        COR c;
        c.other = false;
        c.ind = i;
        const float p1 = std::min(segments_[i](2), segments_[(i+1)%segments_.size()](2))+0.001f;
        const float p2 = o.segments_[(cors[ind].ind-1+o.segments_.size())%o.segments_.size()](2)+0.001f;
        nmi/=nmi.norm();
        const float ramp = std::max((p1-0.001f)/weight_,(p2-0.001f)/o.weight_);
        const float w1 = (!out?1.f:ramp);
        const float w2 = (out?1.f:ramp);
        c.p.head<2>() = w1*p2/(w2*p1+w1*p2)*mi*nmi + segments_[i].head<2>();
        //c.p(2) = (w2*p1+w1*p2-0.002f)/(w1*o.weight_+w2*weight_);
        c.p(2) = (p1+p2-0.002f)/(o.weight_+weight_);
        cors.insert(cors.begin()+ind, c);

        std::cout<<"p1 "<<p1/weight_<<" p2 "<<p2/o.weight_<<" --> "<<c.p(2)<<"\n";
      }

      //      for(size_t i=0; i<segments_.size(); i++)
      //      {
      //        ROS_INFO("%f %f",segments_[i](0),segments_[i](1));
      //      }

      for(size_t j=0; j<cors.size(); j++)
      {
        if(!cors[j].other) {
          //          ROS_INFO("%d", (int)cors[j].ind);
          continue;
        }

        size_t next = j+1, bef = j-1;
        for(size_t i=0; i<cors.size(); i++)
        {
          ROS_ASSERT(i+1!=cors.size());

          next %= cors.size();
          bef += cors.size();
          bef %= cors.size();

          if(!cors[next].other && !cors[bef].other) break;

          if(cors[next].other) next++;
          if(cors[bef].other) bef--;
        }

        ROS_ASSERT(!cors[next].other);
        ROS_ASSERT(!cors[bef].other);

        bef = cors[bef].ind;
        next= cors[next].ind;

        float d;
        Eigen::Vector2f n,x;
        bool out;
        {
          Eigen::Vector2f p1,p2, d1,d2;
          x = o.segments_[cors[j].ind].head<2>();
          p1 = segments_[bef].head<2>();
          p2 = segments_[next].head<2>();

          d1 = (x-p1);
          d2 = p2-p1;
          float f = d1.dot(d2)/d2.squaredNorm();
          if(f>1){
            d = (x-p2).norm();
            n = p2-x;
          }
          else if(f<0) {
            d = d1.norm();
            n = p1-x;
          }
          else {
            d = (d1(0)*d2(1)-d1(1)*d2(0))/(d2).norm();
            n(0) = -(p2(1)-p1(1));
            n(1) = (p2(0)-p1(0));
          }
          //          ROS_INFO("out2 %f",d*(-d2(1)*n(0)+d2(0)*n(1)));
          out = d*(-d2(1)*n(0)+d2(0)*n(1))>-0.001f;
        }

        //        std::cout<<segments_[bef].head<2>()<<"\n";
        //        std::cout<<segments_[next].head<2>()<<"\n";

        const float p1 = std::min(o.segments_[cors[j].ind](2), o.segments_[(cors[j].ind+1)%o.segments_.size()](2))+0.001f;
        const float p2 = segments_[next](2)+0.001f;
        n/=n.norm();
        const float ramp = std::max((p1-0.001f)/o.weight_,(p2-0.001f)/weight_);
        const float w1 = (!out?1.f:ramp);
        const float w2 = (out?1.f:ramp);
        cors[j].p.head<2>() = w1*p2/(w2*p1+w1*p2)*d*n + x;
        //cors[j].p(2) = (w2*p1+w1*p2-0.002f)/(w2*o.weight_+w1*weight_);
        cors[j].p(2) = (p1+p2-0.002f)/(o.weight_+weight_);
        //cors[j].p = o.segments_[cors[j].ind];

        std::cout<<"p1 "<<p1/o.weight_<<" p2 "<<p2/weight_<<" --> "<<cors[j].p(2)<<"\n";
        //        std::cout<<o.segments_[cors[j].ind]<<"\n\n";
        //        std::cout<<n<<"\n\n";
        //        std::cout<<cors[j].p<<"\n\n";
      }

      ROS_ASSERT(cors.size() == (segments_.size()+o.segments_.size()) );

      Outline res;

      for(size_t j=0; j<cors.size(); j++)
      {
        res += cors[j].p;
      }

      return res;
    }
#endif

    Outline operator|(const Outline &o) const
    {

      std::vector<COR> cors;

      for(size_t i=0; i<o.segments_.size(); i++)
      {
        COR c;
        c.other = true;
        c.ind = i;
        cors.push_back(c);
      }

      if(cors.size()<2)
        return *this;

      size_t num_last = cors.size();

      for(size_t i=0; i<segments_.size(); i++)
      {
        bool out = false;
        float mi = std::numeric_limits<float>::max();
        Eigen::Vector2f nmi = Eigen::Vector2f::Zero();
        size_t ind=cors.size();
        for(size_t j=0; j<cors.size(); j++)
        {
          if(!cors[j].other) continue;

          Eigen::Vector2f n;
          Eigen::Vector2f p1,p2, x, d1,d2;
          x = segments_[i].head<2>();
          p1 = o.segments_[cors[j].ind].head<2>();
          p2 = o.segments_[(cors[j].ind+1)%o.segments_.size()].head<2>();

          d1 = (x-p1);
          d2 = p2-p1;
          float f = d1.dot(d2)/d2.squaredNorm();
          float d;
          if(f>1){
            d = (x-p2).norm();
            n = p2-x;
          }
          else if(f<0) {
            d = d1.norm();
            n = p1-x;
          }
          else {
            d = (d1(0)*d2(1)-d1(1)*d2(0))/(d2).norm();
            n(0) = -(p2(1)-p1(1));
            n(1) = (p2(0)-p1(0));
          }

          ROS_INFO("%f %f %f",f,d,mi);

          if(std::abs(d)<std::abs(mi))
          {
            nmi = n;
            mi = d;
            ind = j;
            out = d*(-d2(1)*n(0)+d2(0)*n(1))>0.001f;
            ROS_INFO("out2 %f",d*(-d2(1)*n(0)+d2(0)*n(1)));
          }
        }

        if(out) continue;

        if(cors.size()==ind)
          return Outline();

        ROS_ASSERT(cors.size()!=ind);

        ROS_INFO("-------------");

        for(size_t k=0; k<cors.size(); k++)
        {
          ind = (ind+1)%cors.size();
          ROS_INFO("cmp %d %d", (int)std::abs((int)cors[ind].ind-(int)i), (int)std::abs((int)cors[ind].ind+segments_.size()-(int)i));
          if(cors[ind].other || std::abs((int)cors[ind].ind-(int)i)>std::abs((int)cors[ind].ind+segments_.size()-(int)i))
            break;
        }

        COR c;
        c.other = false;
        c.ind = i;
        const float p1 = std::max(segments_[i](2), segments_[(i+1)%segments_.size()](2))+0.001f;
        const float p2 = o.segments_[(cors[ind].ind-1+o.segments_.size())%o.segments_.size()](2)+0.001f;
        nmi/=nmi.norm();
        c.p.head<2>() = p2*mi*nmi + segments_[i].head<2>();
        c.p(2) = std::max(p1,p2)-0.001f;
        cors.insert(cors.begin()+ind, c);
      }

      if(cors.size()-num_last<2)
        return o;

      for(size_t j=0; j<cors.size(); j++)
      {
        if(!cors[j].other) {
          ROS_INFO("%d", (int)cors[j].ind);
          continue;
        }

        size_t next = j+1, bef = j-1;
        for(size_t i=0; i<cors.size(); i++)
        {
          ROS_ASSERT(i+1!=cors.size());

          next %= cors.size();
          bef += cors.size();
          bef %= cors.size();

          if(!cors[next].other && !cors[bef].other) break;

          if(cors[next].other) next++;
          if(cors[bef].other) bef--;
        }

        ROS_ASSERT(!cors[next].other);
        ROS_ASSERT(!cors[bef].other);

        bef = cors[bef].ind;
        next= cors[next].ind;

        float d;
        Eigen::Vector2f n,x;
        bool out;
        {
          Eigen::Vector2f p1,p2, d1,d2;
          x = o.segments_[cors[j].ind].head<2>();
          p1 = segments_[bef].head<2>();
          p2 = segments_[next].head<2>();

          d1 = (x-p1);
          d2 = p2-p1;
          float f = d1.dot(d2)/d2.squaredNorm();
          if(f>1){
            d = (x-p2).norm();
            n = p2-x;
          }
          else if(f<0) {
            d = d1.norm();
            n = p1-x;
          }
          else {
            d = (d1(0)*d2(1)-d1(1)*d2(0))/(d2).norm();
            n(0) = -(p2(1)-p1(1));
            n(1) = (p2(0)-p1(0));
          }
          //          ROS_INFO("out2 %f",d*(-d2(1)*n(0)+d2(0)*n(1)));
          out = d*(-d2(1)*n(0)+d2(0)*n(1))>0.001f;

          //          ROS_INFO("%f %f",f,d);
        }

        if(out) continue;

        std::cout<<segments_[bef].head<2>()<<"\n";
        std::cout<<segments_[next].head<2>()<<"\n";

        const float p1 = std::max(o.segments_[cors[j].ind](2), o.segments_[(cors[j].ind+1)%o.segments_.size()](2))+0.001f;
        const float p2 = segments_[next](2)+0.001f;
        n/=n.norm();
        cors[j].p.head<2>() = p2*d*n + x;
        cors[j].p(2) = std::max(p1,p2)-0.001f;
        //cors[j].p = o.segments_[cors[j].ind];

        std::cout<<o.segments_[cors[j].ind]<<"\n\n";
        std::cout<<n<<"\n\n";
        std::cout<<cors[j].p<<"\n\n";
      }

      Outline res;

      for(size_t j=0; j<cors.size(); j++)
      {
        res += cors[j].p;
      }

      return res;
    }

    void reverse(const bool fw=true)
    {
      if(segments_.size()<2) return;

      if(fw)
      {
        float t = segments_[segments_.size()-1](2);
        for(size_t i=0; i<segments_.size()-1; i++)
        {
          segments_[(i-1+segments_.size())%segments_.size()](2) = segments_[i](2);
          ROS_INFO("gets %f", segments_[i](2));
        }
        segments_[(segments_.size()-2)%segments_.size()](2) = t;
      }
      else
      {
        float t = segments_[0](2);
        for(size_t i=segments_.size()-1; i>0; i--)
        {
          segments_[(i+1)%segments_.size()](2) = segments_[i](2);
        }
        segments_[1](2) = t;
      }

      std::reverse(segments_.begin(), segments_.end());
    }

    inline size_t size() const {return segments_.size();}
    inline Eigen::Vector3f &operator[](const size_t ind) {return segments_[ind];}
    inline Eigen::Vector3f operator[](const size_t ind) const {return segments_[ind];}
    inline const std::vector<Eigen::Vector3f> &get() const {return segments_;}

    Eigen::Vector3f nextPoint(const Eigen::Vector3f &search) const {
      bool out = false;
      float mi = std::numeric_limits<float>::max();
      Eigen::Vector2f nmi = Eigen::Vector2f::Zero();
      size_t ind=segments_.size();
      for(size_t j=0; j<segments_.size(); j++)
      {
        Eigen::Vector2f n;
        Eigen::Vector2f p1,p2, x, d1,d2;
        x = search.head<2>();
        p1 = segments_[j].head<2>();
        p2 = segments_[(j+1)%segments_.size()].head<2>();

        d1 = (x-p1);
        d2 = p2-p1;
        float f = d1.dot(d2)/d2.squaredNorm();
        float d;
        if(f>1){
          d = (x-p2).norm();
          n = p2-x;
        }
        else if(f<0) {
          d = d1.norm();
          n = p1-x;
        }
        else {
          d = (d1(0)*d2(1)-d1(1)*d2(0))/(d2).norm();
          n(0) = -(p2(1)-p1(1));
          n(1) = (p2(0)-p1(0));
        }

        if(std::abs(d)<std::abs(mi))
        {
          nmi = n;
          mi = d;
          ind = j;
          out = d*(-d2(1)*n(0)+d2(0)*n(1))>-0.001f;
        }
      }

      if(segments_.size()==ind) {
        ROS_ERROR("err X7Z");
        return search;
      }

      Eigen::Vector3f r;
      r(2) = segments_[(ind+segments_.size())%segments_.size()](2);
      nmi/=nmi.norm();
      r.head<2>() = r(2)*mi*nmi + search.head<2>();

      return r;
    }
  };


}


#endif /* FORM_H_ */
