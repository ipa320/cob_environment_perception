/*
 * bb.h
 *
 *  Created on: 15.08.2012
 *      Author: josh
 */

#ifndef BB_H_
#define BB_H_

#include <pcl/common/pca.h>

namespace BoundingBox {
  struct TransformedFoVBB
  {
    struct Plane {
      Eigen::Vector3f normal_x_,normal_y_, offs_;
      Eigen::Vector2f boundary_[4];

      int pnpoly(float x, float y) const
      {
        int i, j, c = 0;
        for (i = 0, j = 4-1; i < 4; j = i++) {
          if ((((boundary_[i](1) <= y) && (y < boundary_[j](1))) ||
              ((boundary_[j](1) <= y) && (y < boundary_[i](1)))) &&
              (x < (boundary_[j](0) - boundary_[i](0)) * (y - boundary_[i](1)) / (boundary_[j](1) - boundary_[i](1)) + boundary_[i](0)))
            c = !c;
        }
        return c;
      }

      bool intersectRay(const Eigen::Vector3f &a, const Eigen::Vector3f &b) const
      {
        Eigen::Matrix3f M;
        Eigen::Vector3f v;
        M.col(0) = a-b;
        M.col(1) = normal_x_;
        M.col(2) = normal_y_;
        //        std::cout<<"M\n"<<M<<"\n";
        v = M.inverse()*(a-offs_);
        //        std::cout<<a<<"\n\n";
        //        std::cout<<b<<"\n\n";
        //        ROS_INFO("%f %f %f", v(0),v(1),v(2));
        if(std::abs(v(0))<=1)
        {
          //          for(int i=0; i<4; i++)
          //            std::cout<<boundary_[i]<<"\n\n";

          return pnpoly(v(1),v(2));
        }
        return false;//TODO: check
      }

      bool sidePt(const Eigen::Vector3f &a) const
      {
        //        std::cout<<"BB\n"<<a<<"\n\n";
        //        std::cout<<offs_<<"\n\n";
        //        std::cout<<normal_x_<<"\n\n";
        //        std::cout<<normal_y_<<"\n\n";
        //        std::cout<<normal_x_.cross(normal_y_)<<"\n\n";
        //        std::cout<<(a-offs_).dot(normal_x_.cross(normal_y_))<<"\n\n";
        return (a-offs_).dot(normal_x_.cross(normal_y_))>0;
      }

    };

    Plane planes_[6];
    Eigen::Vector3f p_[8];

    bool check(const TransformedFoVBB &o) const {
      for(int j=0; j<6; j++)
        for(int i=0; i<4; i++)
        {
          if(planes_[j].intersectRay(o.p_[i],o.p_[(i+1)%4]))
            return true;
          if(planes_[j].intersectRay(o.p_[i+4],o.p_[(i+1)%4+4]))
            return true;
          if(planes_[j].intersectRay(o.p_[i+4],o.p_[i]))
            return true;
        }
      return false;
    }

    bool operator&(const TransformedFoVBB &o) const
    {
      return check(o) || o.check(*this);
    }

    bool operator&(const Eigen::Vector3f &o) const
    {
      if(planes_[0].sidePt(o)!=planes_[1].sidePt(o)) return false;
      if(planes_[2].sidePt(o)!=planes_[4].sidePt(o)) return false;
      if(planes_[3].sidePt(o)!=planes_[5].sidePt(o)) return false;
      return true;
    }
  };



  template<typename _TYPE>
  class FoVBB ///Field of View Bounding Box (as pyramid)
  {
  public:
    typedef _TYPE TYPE;

  protected:
    TYPE min_dist_, max_dist_;
    TYPE min_x_, min_y_, max_x_, max_y_;

  public:

    //for testing
    void update(const Eigen::Vector3f &mi, const Eigen::Vector3f &ma) {
      min_dist_ = min_x_ = min_y_ = std::numeric_limits<float>::max();
      max_dist_ = max_x_ = max_y_ = -std::numeric_limits<float>::max();

      min_dist_ = std::min(min_dist_, mi(2));
      min_x_ = std::min(min_x_, mi(0)/mi(2));
      min_y_ = std::min(min_y_, mi(1)/mi(2));
      max_dist_ = std::max(max_dist_, ma(2));
      max_x_ = std::max(max_x_, ma(0)/ma(2));
      max_y_ = std::max(max_y_, ma(1)/ma(2));
    }

    TransformedFoVBB transform(const Eigen::Matrix3f &R, const Eigen::Vector3f t) const {
      TransformedFoVBB r;
      r.p_[0](0) = min_x_;
      r.p_[0](1) = min_y_;
      r.p_[1](0) = max_x_;
      r.p_[1](1) = min_y_;
      r.p_[2](0) = max_x_;
      r.p_[2](1) = max_y_;
      r.p_[3](0) = min_x_;
      r.p_[3](1) = max_y_;

      Eigen::Vector3f p[8];
      for(int i=0; i<4; i++)
        r.p_[i+4] = r.p_[i];
      for(int i=0; i<8; i++)
      {
        r.p_[i](2) = 1;
        r.p_[i] *= i<4 ? min_dist_:max_dist_;
        p[i] = r.p_[i];
        r.p_[i] = R*r.p_[i] + t;
      }

      Eigen::Vector3f x,y,z;
      x=y=z=Eigen::Vector3f::Zero();
      x(0)=1;
      y(1)=1;
      z(2)=1;

      r.planes_[0].offs_ = R*min_dist_*z+t;
      r.planes_[1].offs_ = R*max_dist_*z+t;
      r.planes_[0].normal_x_ = R*x;
      r.planes_[0].normal_y_ = R*y;
      r.planes_[1].normal_x_ = -R*x;
      r.planes_[1].normal_y_ = R*y;
      for(int i=0; i<4; i++)
      {
        r.planes_[0].boundary_[i] = (p[i].head(2));
        r.planes_[1].boundary_[i] = (p[i+4].head(2));
      }

      //        for(int i=0; i<8; i++)
      //        {
      //          std::cout<<"p\n"<<r.p_[i]<<"\n";
      //        }

      for(int i=0; i<4; i++)
      {
        r.planes_[2+i].normal_x_ = r.p_[i+4]+r.p_[(i+1)%4+4]-r.p_[i]-r.p_[(i+1)%4];
        r.planes_[2+i].normal_x_.normalize();
        r.planes_[2+i].normal_y_ = r.p_[(i+1)%4]-r.p_[i];
        r.planes_[2+i].normal_y_.normalize();

        r.planes_[2+i].boundary_[0](0) = p[i](2);
        r.planes_[2+i].boundary_[1](0) = p[(i+1)%4](2);
        r.planes_[2+i].boundary_[2](0) = p[(i+1)%4](2);
        r.planes_[2+i].boundary_[3](0) = p[i+4](2);

        r.planes_[2+i].boundary_[0](1) = p[i](i%2);
        r.planes_[2+i].boundary_[1](1) = p[(i+1)%4](i%2);
        r.planes_[2+i].boundary_[2](1) = p[(i+1)%4](i%2);
        r.planes_[2+i].boundary_[3](1) = p[i+4](i%2);

        r.planes_[2+i].offs_ = t;
      }

      return r;
    }

    void get8Edges(Eigen::Vector3f *edges) const {
      for(int i=0; i<4; i++) {
        edges[i+4](0) = edges[i](0) = (i&1?min_x_:max_x_);
        edges[i+4](1) = edges[i](1) = (i&2?min_y_:max_y_);
        edges[i+4](2) = edges[i](2) = 1;
        edges[i+4] *= max_dist_;
        edges[i] *= min_dist_;
      }
    }

  };


  class AABB {
    Eigen::Vector3f mi_, ma_;
  public:
    AABB() {}
    AABB(const Eigen::Vector3f &v):mi_(v),ma_(v) {}

    bool operator&(const AABB &o) const {
      return mi_(0)<=o.ma_(0) && ma_(0)>=o.mi_(0) &&
          mi_(1)<=o.ma_(1) && ma_(1)>=o.mi_(1) &&
          mi_(2)<=o.ma_(2) && ma_(2)>=o.mi_(2) ;
    }

    AABB increaseIfNec(const float rot, const float tr) const {
      AABB r = *this;
      return r;
    }

    AABB transform(const Eigen::Matrix3f &R, const Eigen::Vector3f &tr) const {
      AABB r = *this;
      r.mi_ = R*mi_ + tr;
      r.ma_ = R*ma_ + tr;
      return r;
    }

    void get8Edges(Eigen::Vector3f *edges) const {
      for(int i=0; i<8; i++) {
        edges[i](0) = (i&1?mi_(0):ma_(0));
        edges[i](1) = (i&2?mi_(1):ma_(1));
        edges[i](2) = (i&4?mi_(2):ma_(2));
      }
    }

    float extension() const {
      return (ma_-mi_).norm();
    }

    void minmax(Eigen::Vector3f &mi, Eigen::Vector3f &ma) const {
      mi = mi_;
      ma = ma_;
    }
  };

  class OOBB {
    Eigen::Vector3f m_, e_;
    Eigen::Matrix3f axis_;
  public:
    void debug() const {
      std::cout<<"mid\n"<<m_<<"\n";
      std::cout<<"ex\n"<<e_<<"\n";
      std::cout<<"axis\n"<<axis_<<"\n";
    }

    OOBB() {}

    OOBB(const Eigen::Vector3f &v):axis_(Eigen::Matrix3f::Identity()), m_(v) {for(int i=0; i<3; i++) e_(i)=0.001f;}

    void set(const pcl::PointCloud<pcl::PointXYZ> &pc) {
      pcl::PCA<pcl::PointXYZ> pca;
      pca.compute(pc);

      m_ = pca.getMean().head<3>();
      axis_ = pca.getEigenVectors();
      e_ = Eigen::Vector3f::Zero();

      Eigen::Vector3f t;
      for(size_t i=0; i<pc.size(); i++) {
        t = axis_.transpose()*(pc[i].getVector3fMap()-m_);
        t(0)=std::abs(t(0));t(1)=std::abs(t(1));t(2)=std::abs(t(2));
        if(t(0)>e_(0))
          e_(0)=t(0);
        if(t(1)>e_(1))
          e_(1)=t(1);
        if(t(2)>e_(2))
          e_(2)=t(2);
      }

      for(int i=0; i<3; i++) e_(i)=std::max(e_(i),0.001f);
    }

    Eigen::Vector3f getCenter() const {return m_;}
    Eigen::Vector3f getExtension() const {return e_;}
    Eigen::Matrix3f getAxis() const {return axis_;}

    float preassumption(const OOBB &o) const {
      //preassumption
      int m1=0;
      if(e_(1)<e_(m1)) m1=1;
      if(e_(2)<e_(m1)) m1=2;
      int m2=0;
      if(o.e_(1)<o.e_(m2)) m2=1;
      if(o.e_(2)<o.e_(m2)) m2=2;
      return std::abs(axis_.col(m1).dot(o.axis_.col(m2)));
    }

    bool operator&(const OOBB &o) const {
      float ra, rb;
      Eigen::Matrix3f R, AbsR;
      // Compute rotation matrix expressing b in a’s coordinate frame
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          R(i,j) = axis_.col(i).dot(o.axis_.col(j));
      // Compute translation vector t
      Eigen::Vector3f t2 = o.m_-m_, t;
      // Bring translation into a’s coordinate frame
      t(0) = t2.dot(axis_.col(0));
      t(1) = t2.dot(axis_.col(1));
      t(2) = t2.dot(axis_.col(2));
      // Compute common subexpressions. Add in an epsilon term to
      // counteract arithmetic errors when two edges are parallel and
      // their cross product is (near) null (see text for details)
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          AbsR(i,j) = std::abs(R(i,j)) + 0.00001f;
      // Test axes L = A0, L = A1, L = A2
      for (int i = 0; i < 3; i++) {
        ra = e_(i);
        rb = o.e_(0) * AbsR(i,0) + o.e_(1) * AbsR(i,1) + o.e_(2) * AbsR(i,2);
        //std::cout<<"a "<<t(i)<<" "<< ra + rb<<"\n";
        if (std::abs(t(i)) > ra + rb) return false;
      }
      // Test axes L = B0, L = B1, L = B2
      for (int i = 0; i < 3; i++) {
        ra = e_(0) * AbsR(0,i) + e_(1) * AbsR(1,i) + e_(2) * AbsR(2,i);
        rb = o.e_(i);
        //std::cout<<"b "<<t(0)*R(0,i)+t(1)*R(1,i)+t(2)*R(2,i)<<" "<< ra + rb<<"\n";
        if (std::abs(t(0)*R(0,i)+t(1)*R(1,i)+t(2)*R(2,i)) > ra + rb) return false;
      }
      // Test axis L = A0 x B0
      ra = e_(1) * AbsR(2,0) + e_(2) * AbsR(1,0);
      rb = o.e_(1) * AbsR(0,2) + o.e_(2) * AbsR(0,1);
      if (std::abs(t(2) * R(1,0) - t(1) * R(2,0)) > ra + rb) return false;
      // Test axis L = A0 x B1
      ra = e_(1) * AbsR(2,1) + e_(2) * AbsR(1,1);
      rb = o.e_(0) * AbsR(0,2) + o.e_(2) * AbsR(0,0);
      if (std::abs(t(2) * R(1,1) - t(1) * R(2,1)) > ra + rb) return false;
      // Test axis L = A0 x B2
      ra = e_(1) * AbsR(2,2) + e_(2) * AbsR(1,2);
      rb = o.e_(0) * AbsR(0,1) + o.e_(1) * AbsR(0,0);
      if (std::abs(t(2) * R(1,2) - t(1) * R(2,2)) > ra + rb) return false;
      // Test axis L = A1 x B0
      ra = e_(0) * AbsR(2,0) + e_(2) * AbsR(0,0);
      rb = o.e_(1) * AbsR(1,2) + o.e_(2) * AbsR(1,1);
      if (std::abs(t(0) * R(2,0) - t(2) * R(0,0)) > ra + rb) return false;
      // Test axis L = A1 x B1
      ra = e_(0) * AbsR(2,1) + e_(2) * AbsR(0,1);
      rb = o.e_(0) * AbsR(1,2) + o.e_(2) * AbsR(1,0);
      if (std::abs(t(0) * R(2,1) - t(2) * R(0,1)) > ra + rb) return false;
      // Test axis L = A1 x B2
      ra = e_(0) * AbsR(2,2) + e_(2) * AbsR(0,2);
      rb = o.e_(0) * AbsR(1,1) + o.e_(1) * AbsR(1,0);
      if (std::abs(t(0) * R(2,2) - t(2) * R(0,2)) > ra + rb) return false;
      // Test axis L = A2 x B0
      ra = e_(0) * AbsR(1,0) + e_(1) * AbsR(0,0);
      rb = o.e_(1) * AbsR(2,2) + o.e_(2) * AbsR(2,1);
      if (std::abs(t(1) * R(0,0) - t(0) * R(1,0)) > ra + rb) return false;
      // Test axis L = A2 x B1
      ra = e_(0) * AbsR(1,1) + e_(1) * AbsR(0,1);
      rb = o.e_(0) * AbsR(2,2) + o.e_(2) * AbsR(2,0);
      if (std::abs(t(1) * R(0,1) - t(0) * R(1,1)) > ra + rb) return false;
      // Test axis L = A2 x B2
      ra = e_(0) * AbsR(1,2) + e_(1) * AbsR(0,2);
      rb = o.e_(0) * AbsR(2,1) + o.e_(1) * AbsR(2,0);
      if (std::abs(t(1) * R(0,2) - t(0) * R(1,2)) > ra + rb) return false;
      // Since no separating axis is found, the OBBs must be intersecting
      return true;
    }

    OOBB changeSize(const float fact) const {
      OOBB r = *this;
      for(int i=0; i<3; i++)
        r.e_(i) *= fact;
      return r;
    }

    OOBB setMinSize(const float s) const {
      OOBB r = *this;
      for(int i=0; i<3; i++)
        r.e_(i) = std::max(s,r.e_(i));
      return r;
    }

    OOBB increaseIfNec(const float rot, const float tr) const {
      OOBB r = *this;
      Eigen::Vector3f z;
      z(0)=z(1)=0;
      z(2)=1;
      const float a = r.m_.norm()*rot;
      const float l = a + tr;
      for(int i=0; i<3; i++)
        r.e_(i) = std::max(l-a*r.axis_.col(i).dot(z),r.e_(i));
      return r;
    }

    OOBB transform(const Eigen::Matrix3f &R, const Eigen::Vector3f &tr) const {
      OOBB r = *this;
      r.m_ = R*r.m_ + tr;
      r.axis_ = R*r.axis_;
      return r;
    }

    void get8Edges(Eigen::Vector3f *edges) const {
      for(int i=0; i<8; i++)
        edges[i] = m_ + (i&1?1:-1)*e_(0)*axis_.col(0) + (i&2?1:-1)*e_(1)*axis_.col(1) + (i&4?1:-1)*e_(2)*axis_.col(2);
    }

    float extension() const {
      return std::max(e_(0),std::max(e_(1),e_(2)));
    }

    void minmax(Eigen::Vector3f &mi, Eigen::Vector3f &ma) const {
      Eigen::Vector3f e[8];
      get8Edges(e);
      ma = mi = e[0];
      for(size_t i=1; i<8; i++)
      {
        if(e[i](2)>ma(2)) ma = e[i];
        if(e[i](2)<mi(2)) mi = e[i];
      }
    }

    float ratio() const {
      int m1=0;
      if(e_(1)<e_(m1)) m1=1;
      if(e_(2)<e_(m1)) m1=2;
      int m2=(m1+1)%3;
      for(int i=0; i<3; i++) {
        if(i==m1) continue;
        if(e_(i)<e_(m2)) m2=i;
      }

      return e_(m1)/e_(m2);
    }

  };

}

#endif /* BB_H_ */
