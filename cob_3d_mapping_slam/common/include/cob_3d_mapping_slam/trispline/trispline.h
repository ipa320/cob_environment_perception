/*
 * trispline.h
 *
 *  Created on: 25.10.2012
 *      Author: josh
 */

#ifndef TRISPLINE_H_
#define TRISPLINE_H_

#include <Eigen/Core>

#define BIG_V_ 0

namespace ParametricSurface {

  template<size_t IndexA, size_t IndexB, size_t IndexC>
  class TriangleInst
  {
  protected:
    Eigen::Vector3f* pts_;

  public:
    TriangleInst(Eigen::Vector3f* pts_):
      pts_(pts_)
    {}

    inline Eigen::Vector3f tri(const Eigen::Vector3f &bc) {
      return ptA()*bc(0) + ptB()*bc(1) + ptC()*bc(2);
    }

    inline Eigen::Vector3f normal() {
      Eigen::Vector3f r = (ptB()-ptA()).cross(ptC()-ptA());
      r.normalize();
      return r;
    }

    inline Eigen::Vector3f &ptA() {return pts_[IndexA];}
    inline Eigen::Vector3f &ptB() {return pts_[IndexB];}
    inline Eigen::Vector3f &ptC() {return pts_[IndexC];}

  };

  template<int RowSize, int Index>
  class TriangleIndex {
  public:
    enum {a=(Index-1), b=(Index), c=(Index+RowSize)};
  };

  template<int RowSize, int Index>
  class Triangle
  {

    TriangleInst<TriangleIndex<RowSize,Index>::a, TriangleIndex<RowSize,Index>::b, TriangleIndex<RowSize,Index>::c > b;
    Triangle<RowSize, Index-1> tri_;
    Triangle<RowSize-1, Index-1> tri2_;

  public:

    Triangle(Eigen::Vector3f* pts_):
      b(pts_), tri_(pts_), tri2_(pts_+RowSize) {}

  };

  template<int RowSize>
  class Triangle<RowSize,0>
  {
    TriangleInst<TriangleIndex<RowSize,0>::a, TriangleIndex<RowSize,0>::b, TriangleIndex<RowSize,0>::c > b;

  public:

    Triangle(Eigen::Vector3f* pts_):
      b(pts_) {}

  };



  template<int X, int Y>
  class TriangleCoord {
  public:
    enum {x=X, y=Y, ind=y*(y+1)/2+x};
  };

  template<typename Ca, typename Cb, typename Cc, int Round, int Depth>
  class TriangleC {
    TriangleC<
    Ca,
    TriangleCoord<Cb::x-1,Cb::y>,
    TriangleCoord<Cc::x,Cc::y+1>,
    0,
    Depth-1
    > tri_a_;
    TriangleC<
    TriangleCoord<Ca::x+1,Ca::y>,
    Cb,
    TriangleCoord<Cc::x+1,Cc::y+1>,
    0,//1,
    Depth-1
    > tri_b_;
    TriangleC<
    TriangleCoord<Ca::x,Ca::y-1>,
    TriangleCoord<Cb::x-1,Cb::y-1>,
    Cc,
    0,//2,
    Depth-1
    > tri_c_;
  public:

    inline Eigen::Vector4f operator()(const Eigen::Vector3f &bc, const Eigen::Vector4f* pts_) const {
      return tri_a_(bc, pts_)*bc( (0+Round)%3 ) + tri_b_(bc, pts_)*bc( (1+Round)%3 ) + tri_c_(bc, pts_)*bc( (2+Round)%3 );
    }

    inline Eigen::Vector3f normalAt(const Eigen::Vector3f &bc, const Eigen::Vector4f* pts_) const {
      const Eigen::Vector3f a = tri_a_(bc, pts_).head(3), b=tri_b_(bc, pts_).head(3), c=tri_c_(bc, pts_).head(3);
      return (b-a).cross(c-a);
    }

    inline Eigen::Vector3f normalAt2(const Eigen::Vector3f &bc, const Eigen::Vector4f* pts_) const {
      const Eigen::Vector3f a = normalAt(bc, pts_).head(3), b=normalAt(bc, pts_).head(3), c=normalAt(bc, pts_).head(3);
      return (b-a).cross(c-a);
    }

    inline Eigen::Matrix4f normalAtUV(const Eigen::Vector3f &bc, const Eigen::Vector4f* pts_, const Eigen::Vector2f *_uv) const {
      const Eigen::Vector3f a = tri_a_(bc, pts_).head(3), b=tri_b_(bc, pts_).head(3), c=tri_c_(bc, pts_).head(3);
      Eigen::Matrix4f M;

      Eigen::Vector2f uv[3] =
      {
       _uv[0]*bc(0) + (_uv[1]+_uv[0])*0.5f*bc(1) + (_uv[2]+_uv[0])*0.5f*bc(2),
       _uv[1]*bc(1) + (_uv[0]+_uv[1])*0.5f*bc(0) + (_uv[2]+_uv[1])*0.5f*bc(2),
       _uv[2]*bc(2) + (_uv[1]+_uv[2])*0.5f*bc(1) + (_uv[0]+_uv[2])*0.5f*bc(0)
      };

      std::cout<<"uv\n";
      for(int i=0; i<3; i++) std::cout<<uv[i]<<"\n";

      std::cout<<"bc\n"<<bc<<"\n";
      std::cout<<"a\n"<<a<<"\n";
      std::cout<<"b\n"<<b<<"\n";
      std::cout<<"c\n"<<c<<"\n";

      float f = (uv[2](1)-uv[1](1));
      std::cout<<"f1 "<<f<<"\n";
      if(std::abs(f)>0.00001f)
        M.col(1).head<3>() = (uv[2](1)-uv[0](1))/f*(b-c) + c - a;
      else
        M.col(1).head<3>() = b-c;
      f = (uv[2](0)-uv[1](0));
      if(std::abs(f)>0.00001f)
        M.col(2).head<3>() = (uv[2](0)-uv[0](0))/f*(b-c) + c - a;
      else
        M.col(2).head<3>() = b-c;

      {
        const Eigen::Vector3f a = pts_[Ca::ind].head(3), b=pts_[Cb::ind].head(3), c=pts_[Cc::ind].head(3);
        std::cout<<"a\n"<<a<<"\n";
        std::cout<<"b\n"<<b<<"\n";
        std::cout<<"c\n"<<c<<"\n";
        M.col(3).head<3>() = (b-a).cross(c-a);
        M.col(3).head<3>().normalize();
      }
      M.col(1)(3) = M.col(1).head<3>().dot(M.col(3).head<3>());
      M.col(2)(3) = M.col(2).head<3>().dot(M.col(3).head<3>());
      M.col(3).head<3>() = (b-a).cross(c-a);

      return M;
    }

    inline size_t getIndA() const {return Ca::ind;}
    inline size_t getIndB() const {return Cb::ind;}
    inline size_t getIndC() const {return Cc::ind;}

    void print(const Eigen::Vector4f* pts_) const {
      tri_a_.print(pts_);
      tri_b_.print(pts_);
      tri_c_.print(pts_);
    }

  };

  template<typename Ca, typename Cb, typename Cc, int Round>
  class TriangleC<Ca,Cb,Cc,Round,0> {
  public:

    inline Eigen::Vector4f operator()(const Eigen::Vector3f &bc, const Eigen::Vector4f* pts_) const {
      return pts_[Ca::ind]*bc( (0+Round)%3 ) + pts_[Cb::ind]*bc( (1+Round)%3 ) + pts_[Cc::ind]*bc( (2+Round)%3 );
    }

    inline Eigen::Vector4f normalAt(const Eigen::Vector3f &bc, const Eigen::Vector4f* pts_) const {
      const Eigen::Vector4f a = pts_[Ca::ind], b=pts_[Cb::ind], c=pts_[Cc::ind];
      return (b-a).cross(c-a);
    }


    void print(const Eigen::Vector4f* pts_) const {
#if 0
      printf("x y (%d, %d)\n", Ca::x, Ca::y);
      printf("x y (%d, %d)\n", Cb::x, Cb::y);
      printf("x y (%d, %d)\n", Cc::x, Cc::y);
#endif
      printf("adding (%d, %d, %d):\n", Ca::ind, Cb::ind, Cc::ind);
      std::cout<<"  "<<pts_[Ca::ind]<<"\n";
      std::cout<<"  "<<pts_[Cb::ind]<<"\n";
      std::cout<<"  "<<pts_[Cc::ind]<<"\n";
    }
  };


  class _Line {
  public:
    Eigen::Vector3f u, o;

    _Line() {}
    _Line(const Eigen::Vector3f &u, const Eigen::Vector3f &o):u(u),o(o) {}

    inline Eigen::Vector3f at(const float f) const {
      return f*u+o;
    }

    inline Eigen::Vector3f intersection(const Eigen::Vector3f &n, const Eigen::Vector3f &o2, const Eigen::Vector3f &ip) {
      const float d = u.dot(n);
#if DEBUG_OUT_
      std::cout<<"n\n"<<n<<"\n";
      std::cout<<"u\n"<<u<<"\n";
#endif
      if( std::abs(d)< 0.0001f )
        return ip;
      const float f = (o2).dot(n)/(2*d);
#if DEBUG_OUT_
      std::cout<<"f "<<f<<"   "<<u.dot(n)<<"\n";
#endif
      return at( f );
    }
  };

  class _Plane {
  public:
    Eigen::Vector3f n, o;

    _Plane(const Eigen::Vector3f &n, const Eigen::Vector3f &o):n(n),o(o) {}

    inline Eigen::Vector3f intersection(const _Line &l, float &x) {
#if DEBUG_OUT_
      std::cout<<"np\n"<<n<<"\n";
#endif
      const float d=l.u.dot(n);
      if( std::abs(d)< 0.0001f )
        return 0.5f*(o+l.o);
      return l.at( x=(o-l.o).dot(n)/d );
    }

    inline _Line intersection(const _Plane &p) {
      _Line lr( n.cross( p.n ), Eigen::Vector3f::Zero() );
#if 0
      const float dot = n.dot( p.n );
      const float h2 = p.n.dot( p.o-o );
      const float dot2 = (1-dot*dot);
      if(dot2) {
        lr.o =
            ( (-h2*dot)*n + (h2)*p.n )/dot2 + o;
      }
      else
        ROS_ASSERT(0);
#else

      if(lr.u.squaredNorm()<0.0001f) {
        lr.o = o;
        lr.u = p.o-o;
        return lr;
      }
      //ROS_ASSERT_MSG( lr.u.squaredNorm()>0.00001f, "parrallel planes" );

      const float d1 = -n.dot(o);
      const float d2 = -p.n.dot(p.o);

      int m = 0;
      if( std::abs(lr.u(1))>std::abs(lr.u(m)) ) m=1;
      if( std::abs(lr.u(2))>std::abs(lr.u(m)) ) m=2;

      switch (m) {
        case 0:
          lr.o(1) = (d2*n(2) - d1*p.n(2))/ lr.u(0);
          lr.o(2) = (d1*p.n(1) - d2*n(1))/ lr.u(0);
          break;
        case 1:
          lr.o(0) = (d1*p.n(2) - d2*n(2))/ lr.u(1);
          lr.o(2) = (d2*n(0) - d1*p.n(0))/ lr.u(1);
          break;
        case 2:
          lr.o(0) = (d2*n(1) - d1*p.n(1))/ lr.u(2);
          lr.o(1) = (d1*p.n(0) - d2*n(0))/ lr.u(2);
          break;
      }
#endif

#if DEBUG_OUT_
      std::cout<<"PLANE INTERSECTION\n";
      std::cout<<"o\n"<<lr.o<<"\n";
      std::cout<<"u\n"<<lr.u<<"\n";
      std::cout<<"n1\n"<<n<<"\n";
      std::cout<<"n2\n"<<p.n<<"\n";
#endif

      return lr;
    }
  };

  class SplineFade
  {
  public:
    Eigen::Vector4f pts_[2];

    inline Eigen::Vector3f delta() const {return (pts_[0]-pts_[1]).head<3>();}

    void test_setup(const Eigen::Vector3f &a, const Eigen::Vector3f &b,
                    const Eigen::Vector3f &na, const Eigen::Vector3f &nb,
                    const Eigen::Vector3f &na2, const Eigen::Vector3f &nb2,
                    _Line &l1, _Line &l2)
    {
      _Plane p1(na,a);
      _Plane p2(nb,b);

      _Plane p1x(na2,0.5f*(a+b));
      _Plane p2x(nb2,0.5f*(a+b));

      l1 = p1x.intersection(p1);
      l2 = p2x.intersection(p2);

      l1 = _Line(na2,0.5f*(a+b));
      l2 = _Line(nb2,0.5f*(a+b));
    }

    void setup(const Eigen::Vector3f &a, const Eigen::Vector3f &b,
               const Eigen::Vector3f &na, const Eigen::Vector3f &nb,
               const Eigen::Vector3f &na2, const Eigen::Vector3f &nb2,
               const float max_curvature)
    {
      _Plane p1(na,a);
      _Plane p2(nb,b);

#if 0
      Eigen::Vector3f i1 = p1.intersection( _Line(na,b) );
      Eigen::Vector3f i2 = p2.intersection( _Line(nb,a) );
#endif

#if DEBUG_OUT_
      //std::cout<<"i1\n"<<i1<<"\n";
      std::cout<<"a\n"<<a<<"\n";
      //std::cout<<"i2\n"<<i2<<"\n";
      std::cout<<"b\n"<<b<<"\n";
#endif

#if 0
      pts_[0] = _Line( (i1-a), a ).intersection( na2, b-a, i1 );
      pts_[1] = _Line( (i2-b), b ).intersection( nb2, a-b, i2 );
#elif 0
      _Plane p1x(na.cross(a-b),a);
      _Plane p2x(nb.cross(b-a),b);

      pts_[0] = p1x.intersection(p1).intersection( na2, b-a, i1 );
      pts_[1] = p2x.intersection(p2).intersection( nb2, a-b, i2 );
#else

#if 0
      _Plane p1x(na2,0.5f*(a+b));
      _Plane p2x(nb2,0.5f*(a+b));

      _Line l1 = p1x.intersection(p1);
      _Line l2 = p2x.intersection(p2);

      Eigen::Matrix3f M;

#if 0
      M.col(0) = -l1.u;
      M.col(1) = l2.u;
      M.col(2) = l1.u.cross(l2.u);

#if DEBUG_OUT_
      std::cout<<"M\n"<<M<<"\n";
      std::cout<<"r\n"<<(l1.o-l2.o)<<"\n";
      std::cout<<"M-1\n"<<M.inverse()<<"\n";
#endif

      Eigen::Vector3f v=M.inverse()*(l1.o-l2.o);

#if DEBUG_OUT_
      std::cout<<"v\n"<<v<<"\n";
#endif

      if(pcl_isfinite(v.sum())) {
        pts_[0] = l1.at(v(0));
        pts_[1] = l2.at(v(1));
      }
      else {
        pts_[0] = l1.o;
        pts_[1] = l2.o;
      }
#else
      M.col(0) = -l1.u;
      M.col(1) = a-b;
      M.col(2) = l1.u.cross(M.col(1));

      Eigen::Vector3f v=M.inverse()*(l1.o-b);
      std::cout<<"vA\n"<<v<<"\n";
      std::cout<<"u\n"<<l1.u<<"\n";
      std::cout<<"o\n"<<l1.o<<"\n";

      if(pcl_isfinite(v(0)) && std::abs(v(0))<100000.f) {
        pts_[0] = l1.at(v(0));
      }
      else {
        pts_[0] = (a+b)*0.5f;
      }

      M.col(0) = -l2.u;
      M.col(1) = a-b;
      M.col(2) = l2.u.cross(M.col(1));

      v=M.inverse()*(l2.o-b);
      std::cout<<"vB\n"<<v<<"\n";
      std::cout<<"u\n"<<l2.u<<"\n";
      std::cout<<"o\n"<<l2.o<<"\n";

      if(pcl_isfinite(v(0)) && std::abs(v(0))<10000.f) {
        pts_[1] = l2.at(v(0));
      }
      else {
        pts_[1] = (a+b)*0.5f;
      }
#endif

#else

      float x, l;
      pts_[0].head<3>() = p1.intersection(_Line(na2,0.5f*(a+b)),x);
      l = 2*x*x*na2.squaredNorm() / (a-b).squaredNorm();
      if(l>max_curvature*max_curvature){
        pts_[0].head<3>() = (pts_[0].head<3>()-a)*max_curvature/std::sqrt(l) + a;
        //std::cout<<"curv1 "<<l<<" ["<<max_curvature<<std::endl;
      }
      pts_[1].head<3>() = p2.intersection(_Line(nb2,0.5f*(a+b)),x);
      l = 2*x*x*nb2.squaredNorm() / (a-b).squaredNorm();
      if(l>max_curvature*max_curvature) {
        pts_[1].head<3>() = (pts_[1].head<3>()-b)*max_curvature/std::sqrt(l) + b;
        //std::cout<<"curv2 "<<l<<" ["<<max_curvature<<std::endl;
      }

      //      std::cout<<"a   "<<a<<std::endl;
      //      std::cout<<"na  "<<na<<std::endl;
      //      std::cout<<"na2 "<<na2<<std::endl;
      //      std::cout<<"b   "<<b<<std::endl;
      //      std::cout<<"nb  "<<nb<<std::endl;
      //      std::cout<<"nb2 "<<nb2<<std::endl;

      ROS_ASSERT(pcl_isfinite(pts_[0].head<3>().sum()));
      ROS_ASSERT(pcl_isfinite(pts_[1].head<3>().sum()));
#endif

#endif

#if DEBUG_OUT_
      print();
#endif
    }

    void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {
      pts_[0].head<3>() = rot*pts_[0].head<3>()+tr;
      pts_[1].head<3>() = rot*pts_[1].head<3>()+tr;
    }

    inline Eigen::Vector4f operator()(const float a, const float b) {
      float f = a/(a+b);
      if(!pcl_isfinite(f)||f<0) f=0.f;
      if(f>1) f=1;
      return f*pts_[0] + (1.f-f)*pts_[1];
    }

    void print() const {
      std::cout<<"SF1\n"<<pts_[0]<<"\n";
      std::cout<<"SF2\n"<<pts_[1]<<"\n";
    }

  };

  template<int Order>
  class TriSpline
  {
  protected:
    enum {PT_SIZE=(Order+1)*(Order+2)/2, ORDER=Order};

    Eigen::Vector4f pts_[PT_SIZE];

    //Triangle<Order,Order-1> tris_;

    TriangleC<
    TriangleCoord<0,Order>,
    TriangleCoord<Order,Order>,
    TriangleCoord<0,0>,
    0,
    Order-1
    > tri_;

    /*inline Eigen::Vector3f tri(const Eigen::Vector3f &bc) {
      return ptA()*bc(0) + ptB()*bc(1) + ptC()*bc(2);
    }*/

    inline static size_t indAB(const size_t x, const size_t y) {
      return y*(y+1)/2+x;
    }

  public:
    TriSpline()//:tris_(pts_)
    {
      for(int i=0; i<PT_SIZE; i++)
        pts_[i].fill(1);
    }

    virtual ~TriSpline() {}

    virtual Eigen::Vector3f operator()(const Eigen::Vector3f &bc) const {
      return tri_(bc, pts_).head(3);
    }

    virtual Eigen::Vector3f normalAt(const Eigen::Vector3f &bc) const {
      return tri_.normalAt(bc, pts_).head(3);
    }

    virtual Eigen::Vector3f normalAt2(const Eigen::Vector3f &bc) const {
      return tri_.normalAt2(bc, pts_).head(3);
    }

    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {
      for(int i=0; i<PT_SIZE; i++)
        pts_[i].head(3) = rot*pts_[i].head(3)+tr;
    }

  };

  class TriSpline2_Fade : public TriSpline<2> {
    Eigen::Vector3f n_[3], n2_[3];
    Eigen::Vector2f uv_[3];
    SplineFade sf_[3];
    Eigen::Matrix2f _T_;

    Eigen::Vector3f m_, weight_;

    void setup(const float max_curvature) {
      size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

      for(int i=0; i<3; i++) {
        sf_[i].setup( pts_[ind[i]].head<3>(), pts_[ind[(i+1)%3]].head<3>(),
                      n_[i], n_[(i+1)%3],
                      n2_[i], n2_[(i+1)%3], max_curvature);
        m_ += pts_[ind[i]].head<3>();
      }

      m_/=3;
    }

  public:

    void test_setup(_Line *l) {
      size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

      for(int i=0; i<3; i++) {
        sf_[i].test_setup( pts_[ind[i]].head<3>(), pts_[ind[(i+1)%3]].head<3>(),
                           n_[i], n_[(i+1)%3],
                           n2_[i], n2_[(i+1)%3],
                           l[i*2+0],l[i*2+1]);
      }
    }

    inline Eigen::Vector3f getEdge(const int i) {
      static const size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };
      return pts_[ind[i]].head<3>();
    }

    inline SplineFade getFade(const int i) {return sf_[i];}
    inline Eigen::Vector3f getNormal(const int i) {return n_[i];}
    inline Eigen::Vector2f getUV(const int i) {return uv_[i];}
    inline Eigen::Vector3f getNormal2(const int i) {return n2_[i];}

    TriSpline2_Fade(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c,
                    const Eigen::Vector3f &na, const Eigen::Vector3f &nb, const Eigen::Vector3f &nc,
                    const Eigen::Vector3f &na2, const Eigen::Vector3f &nb2, const Eigen::Vector3f &nc2,
                    const Eigen::Vector2f &uva, const Eigen::Vector2f &uvb, const Eigen::Vector2f &uvc,
                    const Eigen::Vector3f &w, const float max_curvature = 10000.f
    ): weight_(w)
    {
      pts_[indAB(0,ORDER)].head<3>()     = a;
      pts_[indAB(ORDER,ORDER)].head<3>() = b;
      pts_[indAB(0,0)].head<3>()         = c;

      n_[0] = na;
      n_[1] = nb;
      n_[2] = nc;

      n2_[0] = na2;
      n2_[1] = nb2;
      n2_[2] = nc2;

      uv_[0] = uva;
      uv_[1] = uvb;
      uv_[2] = uvc;

      //update barycentric coordinates transformation
      _T_.col(0) = uva-uvc;
      _T_.col(1) = uvb-uvc;
      _T_ = _T_.inverse().eval();

      if(!pcl_isfinite(_T_.sum())) {
        ROS_WARN("_T_ is invalid");
        _T_.fill(0);
        _T_(0,0)=_T_(1,1)=1;
      }

      setup(max_curvature);
    }

    inline Eigen::Vector3f getWeight() const {return weight_;}

    virtual Eigen::Vector3f operator()(const Eigen::Vector3f &bc) {
      pts_[indAB(ORDER/2,ORDER)]        = sf_[0](bc(0),bc(1));
      pts_[indAB(ORDER/2,ORDER/2)]      = sf_[1](bc(1),bc(2));
      pts_[indAB(0,ORDER/2)]            = sf_[2](bc(2),bc(0));
#if DEBUG_OUT_
      tri_.print(pts_);
#endif
      return tri_(bc, pts_).head(3);
    }

    virtual Eigen::Vector3f normalAt(const Eigen::Vector3f &bc) {
      pts_[indAB(ORDER/2,ORDER)]        = sf_[0](bc(0),bc(1));
      pts_[indAB(ORDER/2,ORDER/2)]      = sf_[1](bc(1),bc(2));
      pts_[indAB(0,ORDER/2)]            = sf_[2](bc(2),bc(0));
      return tri_.normalAt(bc, pts_);
    }

    virtual Eigen::Vector3f normalAt2(const Eigen::Vector3f &bc) {
      pts_[indAB(ORDER/2,ORDER)]        = sf_[0](bc(0),bc(1));
      pts_[indAB(ORDER/2,ORDER/2)]      = sf_[1](bc(1),bc(2));
      pts_[indAB(0,ORDER/2)]            = sf_[2](bc(2),bc(0));
      return tri_.normalAt2(bc, pts_);
    }

    virtual Eigen::Matrix4f normalAtUV(const Eigen::Vector3f &bc) {
      pts_[indAB(ORDER/2,ORDER)]        = sf_[0](bc(0),bc(1));
      pts_[indAB(ORDER/2,ORDER/2)]      = sf_[1](bc(1),bc(2));
      pts_[indAB(0,ORDER/2)]            = sf_[2](bc(2),bc(0));
      return tri_.normalAtUV(bc, pts_, uv_);
    }

    Eigen::Matrix3f delta() const {
      Eigen::Matrix3f M;
      for(int i=0; i<3; i++)
        M.col(i) = sf_[i].delta();
      return M;
    }

    Eigen::Vector3f operator()(const Eigen::Vector2f &pt) {
      //1. bayrcentric coordinates 2D -> 3D
      Eigen::Vector3f br;
      br.head<2>() = _T_*(pt-uv_[2]);
      br(2) = 1-br(0)-br(1);

      //return (*this)(br);
      Eigen::Vector3f v=(*this)(br);
#if BIG_V_
      if(v.squaredNorm()>70 || !pcl_isfinite(v.sum())) {
        std::cout<<"BIG v\n"<<v<<"\n";
        std::cout<<"uv\n"<<pt<<"\n";
        std::cout<<"bc\n"<<br<<"\n";
        std::cout<<"T\n"<<_T_<<"\n";
        size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

        for(int i=0; i<3; i++) {
          std::cout<<"cp\n"<<pts_[ind[i]]<<"\n";
          std::cout<<"uv\n"<<uv_[i]<<"\n";
        }
      }
#endif
      return v;
    }

    Eigen::Vector3f normalAt(const Eigen::Vector2f &pt) {
      //1. bayrcentric coordinates 2D -> 3D
      Eigen::Matrix<float,3,2> M = normalBC( _T_*(pt-uv_[2]) );

      return M.col(0).cross( M.col(1) );
    }

    Eigen::Vector3f UV2BC(const Eigen::Vector2f &pt) const {
      //1. bayrcentric coordinates 2D -> 3D
      Eigen::Vector3f br;
      br.head<2>() = _T_*(pt-uv_[2]);
      br(2) = 1-br(0)-br(1);

      return br;
    }

    Eigen::Vector3f normalAt2(const Eigen::Vector2f &pt) {
      Eigen::Vector2f v = _T_*(pt-uv_[2]);
      Eigen::Matrix<float,3,2> M1 = normalBC( v );
      Eigen::Matrix<float,3,2> M2 = normal2BC( v );

      M1.col(0) = -( M2.col(0).cross(M1.col(1)) ).cross( M2.col(1).cross(M1.col(0)) );

      //M1.col(0).normalize();

      if(!pcl_isfinite(M1.col(0).sum())) {
        std::cout<<"INF v\n"<<M1<<"\n";
        std::cout<<"M1\n"<<normalBC( v )<<"\n";
        std::cout<<"M2\n"<<normal2BC( v )<<"\n";
        std::cout<<"uv\n"<<pt<<"\n";
        std::cout<<"bc\n"<<v<<"\n";
        std::cout<<"T\n"<<_T_<<"\n";
        size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

        for(int i=0; i<3; i++) {
          std::cout<<"cp\n"<<pts_[ind[i]]<<"\n";
          std::cout<<"uv\n"<<uv_[i]<<"\n";
        }
      }

      if(!pcl_isfinite(M1.col(0).sum()))
        return Eigen::Vector3f::Zero();

      return M1.col(0);
      /*
      //1. bayrcentric coordinates 2D -> 3D
      Eigen::Matrix3f M;

      M.topLeftCorner<3,2>() = normal2BC(_T_*(pt-uv_[2]));
      M.col(2) = M.col(0).cross(M.col(1));

      Eigen::Matrix<float,3,2> R;
      Eigen::Vector3f v;
      v(2)=0;

      v.head<2>() = _T_.col(0);
      R.col(0) = M*v;

      v.head<2>() = _T_.col(1);
      R.col(1) = M*v;

      std::cout<<"normalAt2\n"<<R<<"\n";
      std::cout<<"normalAt2v\n"<<R*pt<<"\n";

      return R*pt;*/
    }

    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {
      TriSpline<2>::transform(rot, tr);
      for(int i=0; i<3; i++) {
        //std::cout<<tr;
        n_[i]  = rot*n_[i];
        n2_[i] = rot*n2_[i];
        sf_[i].transform(rot,tr);
      }

      m_ = rot*m_ + tr;
    }

    inline Eigen::Vector3f getMid() const {return m_;}

    inline Eigen::Vector2f getMidUV() const {return (uv_[0]+uv_[1]+uv_[2])/3;}

    Eigen::Matrix4f normalAtUV(const Eigen::Vector2f &pt) {
      for(int i=0; i<3; i++)
        sf_[i].print();

      //1. bayrcentric coordinates 2D -> 3D
      Eigen::Vector3f br;

      br.head<2>() = _T_*(pt-uv_[2]);
      br(2) = 1-br(0)-br(1);

      Eigen::Matrix4f M = normalAtUV(br);
      M.col(0).head<3>() = (*this)(br);
      //      M.col(1).head<3>().normalize();
      //      M.col(2).head<3>().normalize();
      return M;
    }

    inline float POW2(const float f) {return f*f;}
    inline float POW3(const float f) {return f*f*f;}

    inline Eigen::Matrix<float,3,2> normal(const Eigen::Vector2f &uv) {
      Eigen::Matrix3f M;
      M.topLeftCorner<3,2>() = normalBC(_T_*(uv-uv_[2]));
      M.col(2) = M.col(0).cross(M.col(1));

      Eigen::Matrix<float,3,2> R;
      Eigen::Vector3f v;
      v(2)=0;

      v.head<2>() = _T_.col(0);
      R.col(0) = M*v;

      v.head<2>() = _T_.col(1);
      R.col(1) = M*v;

      if(!pcl_isfinite(R.sum())) {
        std::cout<<"INF v\n"<<R<<"\n";
        std::cout<<"uv\n"<<uv<<"\n";
        std::cout<<"bc\n"<<(_T_*(uv-uv_[2]))<<"\n";
        std::cout<<"T\n"<<_T_<<"\n";
        size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

        for(int i=0; i<3; i++) {
          std::cout<<"cp\n"<<pts_[ind[i]]<<"\n";
          std::cout<<"uv\n"<<uv_[i]<<"\n";
        }
      }

      return R;
    }

    inline Eigen::Matrix<float,3,2> normalBC(const Eigen::Vector2f &bc) {
      Eigen::Matrix<float,3,2> M;
      static const size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

#if 0
      if(std::abs(bc(0)-1)<0.0001f) {
        //        M.col(0) = 2*pts_[ind[0]]-2*sf_[2].pts_[1];
        //        M.col(1) = 2*sf_[0].pts_[0]-2*sf_[2].pts_[1];

        M.col(0) = -(-bc(1)-bc(0)+1)*pts_[ind[2]]+(-bc(1)-bc(0)+1)*(-pts_[ind[2]]+bc(0)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+(bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))+bc(0)*((-bc(1)-bc(0)+1)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-(bc(0)*sf_[2].pts_[1])/(1-bc(1))-((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+pts_[ind[0]])+(-bc(1)-bc(0)+1)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))-bc(0)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))+bc(1)*(-(sf_[1].pts_[1]+sf_[1].pts_[0])/2+bc(0)*(-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+(bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))-(bc(1)*(sf_[1].pts_[1]+sf_[1].pts_[0]))/2+bc(1)*((bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))+bc(0)*pts_[ind[0]];
        M.col(1) = -(-bc(1)-bc(0)+1)*pts_[ind[2]]+(-bc(1)-bc(0)+1)*(-pts_[ind[2]]+bc(0)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+(sf_[1].pts_[1]+sf_[1].pts_[0])/2)+bc(1)*(pts_[ind[1]]-(sf_[1].pts_[1]+sf_[1].pts_[0])/2+bc(0)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0))))+bc(1)*pts_[ind[1]]+bc(0)*((-bc(1)-bc(0)+1)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-(bc(0)*sf_[2].pts_[1])/(1-bc(1))-((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+(bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))-bc(0)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))-(bc(1)*(sf_[1].pts_[1]+sf_[1].pts_[0]))/2+((-bc(1)-bc(0)+1)*(sf_[1].pts_[1]+sf_[1].pts_[0]))/2+bc(0)*((bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)));

        //std::cout<<"A\n"<<M<<"\n";
      }
      else if(std::abs(bc(1)-1)<0.0001f) {
        //        M.col(0) = 2*sf_[0].pts_[1]-2*sf_[1].pts_[0];
        //        M.col(1) = 2*pts_[ind[1]]-2*sf_[1].pts_[0];

        M.col(0) = -(-bc(1)-bc(0)+1)*pts_[ind[2]]+(-bc(1)-bc(0)+1)*(-pts_[ind[2]]+(sf_[2].pts_[1]+sf_[2].pts_[0])/2+bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0))))+bc(0)*(-(sf_[2].pts_[1]+sf_[2].pts_[0])/2+bc(1)*(-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+pts_[ind[0]])+((-bc(1)-bc(0)+1)*(sf_[2].pts_[1]+sf_[2].pts_[0]))/2-(bc(0)*(sf_[2].pts_[1]+sf_[2].pts_[0]))/2+bc(1)*((-bc(1)-bc(0)+1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0)))-((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(bc(1)*sf_[1].pts_[0])/(1-bc(0))+bc(0)*(-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+(bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))-bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+bc(1)*((bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))+bc(0)*pts_[ind[0]];
        M.col(1) = -(-bc(1)-bc(0)+1)*pts_[ind[2]]+(-bc(1)-bc(0)+1)*(-pts_[ind[2]]+bc(1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))+((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+bc(1)*(pts_[ind[1]]+(-bc(1)-bc(0)+1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))-((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(bc(1)*sf_[1].pts_[0])/(1-bc(0))+bc(0)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0))))+bc(1)*pts_[ind[1]]+bc(0)*(-(sf_[2].pts_[1]+sf_[2].pts_[0])/2+bc(1)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+(bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))-(bc(0)*(sf_[2].pts_[1]+sf_[2].pts_[0]))/2-bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+(-bc(1)-bc(0)+1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+bc(0)*((bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)));

        //std::cout<<"B\n"<<M<<"\n";
      }
      else if( std::abs(bc(0)+bc(1))<0.0001f) {
        //        M.col(0) = 2*sf_[2].pts_[0]-2*pts_[ind[2]];
        //        M.col(1) = 2*sf_[1].pts_[1]-2*pts_[ind[2]];

        M.col(0) = -(-bc(1)-bc(0)+1)*pts_[ind[2]]+(-bc(1)-bc(0)+1)*(-pts_[ind[2]]+bc(0)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+(bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0))))+bc(0)*((-bc(1)-bc(0)+1)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-(bc(0)*sf_[2].pts_[1])/(1-bc(1))-((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+pts_[ind[0]])+(-bc(1)-bc(0)+1)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))-bc(0)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))+bc(1)*((-bc(1)-bc(0)+1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0)))-((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(bc(1)*sf_[1].pts_[0])/(1-bc(0))+(sf_[0].pts_[1]+sf_[0].pts_[0])/2)-bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+(bc(1)*(sf_[0].pts_[1]+sf_[0].pts_[0]))/2+bc(0)*pts_[ind[0]];
        M.col(1) = -(-bc(1)-bc(0)+1)*pts_[ind[2]]+(-bc(1)-bc(0)+1)*(-pts_[ind[2]]+bc(0)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+bc(1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))+((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+bc(1)*(pts_[ind[1]]+(-bc(1)-bc(0)+1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))-((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+bc(1)*pts_[ind[1]]+bc(0)*((-bc(1)-bc(0)+1)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-(bc(0)*sf_[2].pts_[1])/(1-bc(1))-((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+(sf_[0].pts_[1]+sf_[0].pts_[0])/2)-bc(0)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))-bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+(-bc(1)-bc(0)+1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+(bc(0)*(sf_[0].pts_[1]+sf_[0].pts_[0]))/2;

        //std::cout<<"C\n"<<M<<"\n";
      }
      else {
        M.col(0) = -(-bc(1)-bc(0)+1)*pts_[ind[2]]+(-bc(1)-bc(0)+1)*(-pts_[ind[2]]+bc(0)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+(bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0))))+bc(0)*
            ((-bc(1)-bc(0)+1)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-(bc(0)*sf_[2].pts_[1])/(1-bc(1))-((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+pts_[ind[0]])+(-bc(1)-bc(0)+1)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))-bc(0)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))+bc(1)*
            ((-bc(1)-bc(0)+1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0)))-((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(bc(1)*sf_[1].pts_[0])/(1-bc(0))+bc(0)*(-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+(bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))-bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+bc(1)*((bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))+bc(0)*pts_[ind[0]];
        M.col(1) = -(-bc(1)-bc(0)+1)*pts_[ind[2]]+(-bc(1)-bc(0)+1)*(-pts_[ind[2]]+bc(0)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+bc(1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))+((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+bc(1)*
            (pts_[ind[1]]+(-bc(1)-bc(0)+1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))-((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(bc(1)*sf_[1].pts_[0])/(1-bc(0))+bc(0)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0))))+bc(1)*pts_[ind[1]]+bc(0)*
            ((-bc(1)-bc(0)+1)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-(bc(0)*sf_[2].pts_[1])/(1-bc(1))-((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+(bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)))-bc(0)*((bc(0)*sf_[2].pts_[1])/(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1)))-bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+
            (-bc(1)-bc(0)+1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/(1-bc(0)))+bc(0)*((bc(1)*sf_[0].pts_[1])/(bc(1)+bc(0))+(bc(0)*sf_[0].pts_[0])/(bc(1)+bc(0)));
      }
#else
      const float a=bc(0), b=bc(1);

      if(std::abs(a-1)<0.001f) {
        //        M.col(0) = 2*pts_[ind[0]]-2*sf_[2].pts_[1];
        //        M.col(1) = 2*sf_[0].pts_[0]-2*sf_[2].pts_[1];

        M.col(0) = ( -(-b-a+1)*pts_[ind[2]]+(-b-a+1)*(-pts_[ind[2]]+a*(sf_[2].pts_[1]/(1-b)-sf_[2].pts_[0]/(1-b))+(a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))+a*((-b-a+1)*(sf_[2].pts_[1]/(1-b)-sf_[2].pts_[0]/(1-b))-(a*sf_[2].pts_[1])/(1-b)-((-b-a+1)*sf_[2].pts_[0])/(1-b)+b*(-(b*sf_[0].pts_[1])/POW2(b+a)+sf_[0].pts_[0]/(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+pts_[ind[0]])+(-b-a+1)*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))-a*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))+b*(-(sf_[1].pts_[1]+sf_[1].pts_[0])/2+a*(-(b*sf_[0].pts_[1])/POW2(b+a)+sf_[0].pts_[0]/(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+(b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))-(b*(sf_[1].pts_[1]+sf_[1].pts_[0]))/2+b*((b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))+a*pts_[ind[0]] ).head<3>();
        M.col(1) = ( -(-b-a+1)*pts_[ind[2]]+(-b-a+1)*(-pts_[ind[2]]+a*((a*sf_[2].pts_[1])/POW2(1-b)+((-b-a+1)*sf_[2].pts_[0])/POW2(1-b)-sf_[2].pts_[0]/(1-b))+(sf_[1].pts_[1]+sf_[1].pts_[0])/2)+b*(pts_[ind[1]]-(sf_[1].pts_[1]+sf_[1].pts_[0])/2+a*(sf_[0].pts_[1]/(b+a)-(b*sf_[0].pts_[1])/POW2(b+a)-(a*sf_[0].pts_[0])/POW2(b+a)))+b*pts_[ind[1]]+a*((-b-a+1)*((a*sf_[2].pts_[1])/POW2(1-b)+((-b-a+1)*sf_[2].pts_[0])/POW2(1-b)-sf_[2].pts_[0]/(1-b))-(a*sf_[2].pts_[1])/(1-b)-((-b-a+1)*sf_[2].pts_[0])/(1-b)+b*(sf_[0].pts_[1]/(b+a)-(b*sf_[0].pts_[1])/POW2(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+(b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))-a*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))-(b*(sf_[1].pts_[1]+sf_[1].pts_[0]))/2+((-b-a+1)*(sf_[1].pts_[1]+sf_[1].pts_[0]))/2+a*((b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a)) ).head<3>();

        //std::cout<<"A\n"<<M<<"\n";
      }
      else if(std::abs(b-1)<0.001f) {
        //        M.col(0) = 2*sf_[0].pts_[1]-2*sf_[1].pts_[0];
        //        M.col(1) = 2*pts_[ind[1]]-2*sf_[1].pts_[0];

        M.col(0) = ( -(-b-a+1)*pts_[ind[2]]+(-b-a+1)*(-pts_[ind[2]]+(sf_[2].pts_[1]+sf_[2].pts_[0])/2+b*(((-b-a+1)*sf_[1].pts_[1])/POW2(1-a)-sf_[1].pts_[1]/(1-a)+(b*sf_[1].pts_[0])/POW2(1-a)))+a*(-(sf_[2].pts_[1]+sf_[2].pts_[0])/2+b*(-(b*sf_[0].pts_[1])/POW2(b+a)+sf_[0].pts_[0]/(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+pts_[ind[0]])+((-b-a+1)*(sf_[2].pts_[1]+sf_[2].pts_[0]))/2-(a*(sf_[2].pts_[1]+sf_[2].pts_[0]))/2+b*((-b-a+1)*(((-b-a+1)*sf_[1].pts_[1])/POW2(1-a)-sf_[1].pts_[1]/(1-a)+(b*sf_[1].pts_[0])/POW2(1-a))-((-b-a+1)*sf_[1].pts_[1])/(1-a)-(b*sf_[1].pts_[0])/(1-a)+a*(-(b*sf_[0].pts_[1])/POW2(b+a)+sf_[0].pts_[0]/(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+(b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))-b*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+b*((b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))+a*pts_[ind[0]] ).head<3>();
        M.col(1) = ( -(-b-a+1)*pts_[ind[2]]+(-b-a+1)*(-pts_[ind[2]]+b*(sf_[1].pts_[0]/(1-a)-sf_[1].pts_[1]/(1-a))+((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+b*(pts_[ind[1]]+(-b-a+1)*(sf_[1].pts_[0]/(1-a)-sf_[1].pts_[1]/(1-a))-((-b-a+1)*sf_[1].pts_[1])/(1-a)-(b*sf_[1].pts_[0])/(1-a)+a*(sf_[0].pts_[1]/(b+a)-(b*sf_[0].pts_[1])/POW2(b+a)-(a*sf_[0].pts_[0])/POW2(b+a)))+b*pts_[ind[1]]+a*(-(sf_[2].pts_[1]+sf_[2].pts_[0])/2+b*(sf_[0].pts_[1]/(b+a)-(b*sf_[0].pts_[1])/POW2(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+(b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))-(a*(sf_[2].pts_[1]+sf_[2].pts_[0]))/2-b*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+(-b-a+1)*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+a*((b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a)) ).head<3>();

        //std::cout<<"B\n"<<M<<"\n";
      }
      else if( std::abs(a+b)<0.003f) {
        //        M.col(0) = 2*sf_[2].pts_[0]-2*pts_[ind[2]];
        //        M.col(1) = 2*sf_[1].pts_[1]-2*pts_[ind[2]];

        M.col(0) = ( -(-b-a+1)*pts_[ind[2]]+(-b-a+1)*(-pts_[ind[2]]+a*(sf_[2].pts_[1]/(1-b)-sf_[2].pts_[0]/(1-b))+(a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b)+b*(((-b-a+1)*sf_[1].pts_[1])/POW2(1-a)-sf_[1].pts_[1]/(1-a)+(b*sf_[1].pts_[0])/POW2(1-a)))+a*((-b-a+1)*(sf_[2].pts_[1]/(1-b)-sf_[2].pts_[0]/(1-b))-(a*sf_[2].pts_[1])/(1-b)-((-b-a+1)*sf_[2].pts_[0])/(1-b)+pts_[ind[0]])+(-b-a+1)*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))-a*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))+b*((-b-a+1)*(((-b-a+1)*sf_[1].pts_[1])/POW2(1-a)-sf_[1].pts_[1]/(1-a)+(b*sf_[1].pts_[0])/POW2(1-a))-((-b-a+1)*sf_[1].pts_[1])/(1-a)-(b*sf_[1].pts_[0])/(1-a)+(sf_[0].pts_[1]+sf_[0].pts_[0])/2)-b*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+(b*(sf_[0].pts_[1]+sf_[0].pts_[0]))/2+a*pts_[ind[0]] ).head<3>();
        M.col(1) = ( -(-b-a+1)*pts_[ind[2]]+(-b-a+1)*(-pts_[ind[2]]+a*((a*sf_[2].pts_[1])/POW2(1-b)+((-b-a+1)*sf_[2].pts_[0])/POW2(1-b)-sf_[2].pts_[0]/(1-b))+b*(sf_[1].pts_[0]/(1-a)-sf_[1].pts_[1]/(1-a))+((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+b*(pts_[ind[1]]+(-b-a+1)*(sf_[1].pts_[0]/(1-a)-sf_[1].pts_[1]/(1-a))-((-b-a+1)*sf_[1].pts_[1])/(1-a)-(b*sf_[1].pts_[0])/(1-a))+b*pts_[ind[1]]+a*((-b-a+1)*((a*sf_[2].pts_[1])/POW2(1-b)+((-b-a+1)*sf_[2].pts_[0])/POW2(1-b)-sf_[2].pts_[0]/(1-b))-(a*sf_[2].pts_[1])/(1-b)-((-b-a+1)*sf_[2].pts_[0])/(1-b)+(sf_[0].pts_[1]+sf_[0].pts_[0])/2)-a*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))-b*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+(-b-a+1)*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+(a*(sf_[0].pts_[1]+sf_[0].pts_[0]))/2 ).head<3>();

        //std::cout<<"C\n"<<M<<"\n";
      }
      else {
#if 0
        M.col(0) = ( -(-b-a+1)*pts_[ind[2]]+(-b-a+1)*(-pts_[ind[2]]+a*(sf_[2].pts_[1]/(1-b)-sf_[2].pts_[0]/(1-b))+(a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b)+b*(((-b-a+1)*sf_[1].pts_[1])/POW2(1-a)-sf_[1].pts_[1]/(1-a)+(b*sf_[1].pts_[0])/POW2(1-a)))+a*
            ((-b-a+1)*(sf_[2].pts_[1]/(1-b)-sf_[2].pts_[0]/(1-b))-(a*sf_[2].pts_[1])/(1-b)-((-b-a+1)*sf_[2].pts_[0])/(1-b)+b*(-(b*sf_[0].pts_[1])/POW2(b+a)+sf_[0].pts_[0]/(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+pts_[ind[0]])+(-b-a+1)*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))-a*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))+b*
            ((-b-a+1)*(((-b-a+1)*sf_[1].pts_[1])/POW2(1-a)-sf_[1].pts_[1]/(1-a)+(b*sf_[1].pts_[0])/POW2(1-a))-((-b-a+1)*sf_[1].pts_[1])/(1-a)-(b*sf_[1].pts_[0])/(1-a)+a*(-(b*sf_[0].pts_[1])/POW2(b+a)+sf_[0].pts_[0]/(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+(b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))-b*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+b*((b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))+a*pts_[ind[0]] ).head<3>();
        M.col(1) = ( -(-b-a+1)*pts_[ind[2]]+(-b-a+1)*(-pts_[ind[2]]+a*((a*sf_[2].pts_[1])/POW2(1-b)+((-b-a+1)*sf_[2].pts_[0])/POW2(1-b)-sf_[2].pts_[0]/(1-b))+b*(sf_[1].pts_[0]/(1-a)-sf_[1].pts_[1]/(1-a))+((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+b*
            (pts_[ind[1]]+(-b-a+1)*(sf_[1].pts_[0]/(1-a)-sf_[1].pts_[1]/(1-a))-((-b-a+1)*sf_[1].pts_[1])/(1-a)-(b*sf_[1].pts_[0])/(1-a)+a*(sf_[0].pts_[1]/(b+a)-(b*sf_[0].pts_[1])/POW2(b+a)-(a*sf_[0].pts_[0])/POW2(b+a)))+b*pts_[ind[1]]+a*
            ((-b-a+1)*((a*sf_[2].pts_[1])/POW2(1-b)+((-b-a+1)*sf_[2].pts_[0])/POW2(1-b)-sf_[2].pts_[0]/(1-b))-(a*sf_[2].pts_[1])/(1-b)-((-b-a+1)*sf_[2].pts_[0])/(1-b)+b*(sf_[0].pts_[1]/(b+a)-(b*sf_[0].pts_[1])/POW2(b+a)-(a*sf_[0].pts_[0])/POW2(b+a))+(b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a))-a*((a*sf_[2].pts_[1])/(1-b)+((-b-a+1)*sf_[2].pts_[0])/(1-b))-b*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+
            (-b-a+1)*(((-b-a+1)*sf_[1].pts_[1])/(1-a)+(b*sf_[1].pts_[0])/(1-a))+a*((b*sf_[0].pts_[1])/(b+a)+(a*sf_[0].pts_[0])/(b+a)) ).head<3>();
#else

        const float a2=a*a, b2=b*b;
        const float a3=a2*a, b3=b2*b;

        M.col(0) = M.col(1) = (2*b*pts_[ind[2]]+2*a*pts_[ind[2]]-2*pts_[ind[2]] ).head<3>();

        M.col(0) += (
            ((4*a*b+6*a2-4*a)*sf_[2].pts_[1]+(-2*b2+(4-8*a)*b-6*a2+8*a-2)*sf_[2].pts_[0])/(b-1)
            +(2*b2*sf_[0].pts_[1]+4*a*b*sf_[0].pts_[0])/(b+a)
            -(2*a*b2*sf_[0].pts_[1]+2*a2*b*sf_[0].pts_[0])/(b2+2*a*b+a2)
            -((4*b2+(4*a-4)*b)*sf_[1].pts_[1]-2*b2*sf_[1].pts_[0])/(a-1)
            +((2*b3+(4*a-4)*b2+(2*a2-4*a+2)*b)*sf_[1].pts_[1]+((2-2*a)*b2-2*b3)*sf_[1].pts_[0])/(a2-2*a+1)
            +2*a*pts_[ind[0]]
        ).head<3>();
        M.col(1) += (
            (2*a2*sf_[2].pts_[1]+(-4*a*b-4*a2+4*a)*sf_[2].pts_[0])/(b-1)
            +(4*a*b*sf_[0].pts_[1]+2*a2*sf_[0].pts_[0])/(b+a)
            -(2*a*b2*sf_[0].pts_[1]+2*a2*b*sf_[0].pts_[0])/(b2+2*a*b+a2)
            -((6*b2+(8*a-8)*b+2*a2-4*a+2)*sf_[1].pts_[1]+((4-4*a)*b-6*b2)*sf_[1].pts_[0])/(a-1)
            -((2*a2*b+2*a3-2*a2)*sf_[2].pts_[1]+(-2*a*b2+(4*a-4*a2)*b-2*a3+4*a2-2*a)*sf_[2].pts_[0])/(b2-2*b+1)
            +2*b*pts_[ind[1]]
        ).head<3>();
#endif
      }
#endif

      if(!pcl_isfinite(M.sum())) {
        std::cout<<"INF2 v\n"<<M<<"\n";
        std::cout<<"bc\n"<<bc<<"\n";
        std::cout<<"T\n"<<_T_<<"\n";
        size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

        for(int i=0; i<3; i++) {
          std::cout<<"cp\n"<<pts_[ind[i]]<<"\n";
          std::cout<<"uv\n"<<uv_[i]<<"\n";
        }
      }

      return M;
    }

    inline Eigen::Matrix<float,3,2> normal2BC(const Eigen::Vector2f &bc) {
      //      if(std::abs(bc(0)-1)<0.0010f)
      //        return n2_[0];
      //      else if(std::abs(bc(1)-1)<0.0010f)
      //        return n2_[1];
      //      else if( std::abs(bc(0))+std::abs(bc(1))<0.0001f)
      //        return n2_[2];

      static const size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

      Eigen::Matrix<float,3,2> M;
      if(std::abs(bc(0)-1)<0.001f) {
        M.col(0) = (2*pts_[ind[2]]+(-bc(1)-bc(0)+1)*((2*sf_[2].pts_[1])/(1-bc(1))-(2*sf_[2].pts_[0])/(1-bc(1)))+2*(-bc(1)-bc(0)+1)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-2*bc(0)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+bc(0)*(-(2*sf_[2].pts_[1])/(1-bc(1))+(2*sf_[2].pts_[0])/(1-bc(1))+bc(1)*((2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))-(2*sf_[0].pts_[0])/POW2(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0))))-(4*bc(0)*sf_[2].pts_[1])/(1-bc(1))-(4*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(bc(0)*((2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))-(2*sf_[0].pts_[0])/POW2(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0)))-(2*bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*sf_[0].pts_[0])/(bc(1)+bc(0))-(2*bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+2*bc(1)*(-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+2*pts_[ind[0]] ).head<3>();
        M.col(1) = (2*pts_[ind[2]]+2*pts_[ind[1]]+bc(0)*((-bc(1)-bc(0)+1)*((2*bc(0)*sf_[2].pts_[1])/POW3(1-bc(1))+(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW3(1-bc(1))-(2*sf_[2].pts_[0])/POW2(1-bc(1)))-(2*bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))-(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))+(2*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(-(2*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0)))+(2*sf_[0].pts_[1])/(bc(1)+bc(0))-(2*bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(2*bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))-2*bc(0)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+bc(0)*(-bc(1)-bc(0)+1)*((2*bc(0)*sf_[2].pts_[1])/POW3(1-bc(1))+(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW3(1-bc(1))-(2*sf_[2].pts_[0])/POW2(1-bc(1)))-2*sf_[1].pts_[1]-2*sf_[1].pts_[0]+2*bc(0)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+bc(0)*bc(1)*(-(2*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0))) ).head<3>();

        //std::cout<<"A\n"<<M<<"\n";
      }
      else if(std::abs(bc(1)-1)<0.001f) {
        M.col(0) = (2*pts_[ind[2]]-2*sf_[2].pts_[1]-2*sf_[2].pts_[0]+bc(1)*((-bc(1)-bc(0)+1)*((2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW3(1-bc(0))-(2*sf_[1].pts_[1])/POW2(1-bc(0))+(2*bc(1)*sf_[1].pts_[0])/POW3(1-bc(0)))-(2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))+(2*sf_[1].pts_[1])/(1-bc(0))-(2*bc(1)*sf_[1].pts_[0])/POW2(1-bc(0))+bc(0)*((2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))-(2*sf_[0].pts_[0])/POW2(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0)))-(2*bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*sf_[0].pts_[0])/(bc(1)+bc(0))-(2*bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))-2*bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0)))+(-bc(1)-bc(0)+1)*bc(1)*((2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW3(1-bc(0))-(2*sf_[1].pts_[1])/POW2(1-bc(0))+(2*bc(1)*sf_[1].pts_[0])/POW3(1-bc(0)))+2*bc(1)*(-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+bc(0)*bc(1)*((2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))-(2*sf_[0].pts_[0])/POW2(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0)))+2*pts_[ind[0]]).head<3>();
        M.col(1) = (2*pts_[ind[2]]+2*pts_[ind[1]]+bc(1)*((2*sf_[1].pts_[1])/(1-bc(0))-(2*sf_[1].pts_[0])/(1-bc(0))+bc(0)*(-(2*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0))))-2*bc(1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))+2*(-bc(1)-bc(0)+1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))+(-bc(1)-bc(0)+1)*((2*sf_[1].pts_[0])/(1-bc(0))-(2*sf_[1].pts_[1])/(1-bc(0)))-(4*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(4*bc(1)*sf_[1].pts_[0])/(1-bc(0))+bc(0)*(bc(1)*(-(2*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0)))+(2*sf_[0].pts_[1])/(bc(1)+bc(0))-(2*bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(2*bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+2*bc(0)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))).head<3>();

        //std::cout<<"B\n"<<M<<"\n";
      }
      else if( std::abs(bc(0)+bc(1))<0.001f) {
        M.col(0) = (2*pts_[ind[2]]+(-bc(1)-bc(0)+1)*((2*sf_[2].pts_[1])/(1-bc(1))-(2*sf_[2].pts_[0])/(1-bc(1))+bc(1)*((2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW3(1-bc(0))-(2*sf_[1].pts_[1])/POW2(1-bc(0))+(2*bc(1)*sf_[1].pts_[0])/POW3(1-bc(0))))+2*(-bc(1)-bc(0)+1)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-2*bc(0)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+bc(0)*((2*sf_[2].pts_[0])/(1-bc(1))-(2*sf_[2].pts_[1])/(1-bc(1)))-(4*bc(0)*sf_[2].pts_[1])/(1-bc(1))-(4*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*((-bc(1)-bc(0)+1)*((2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW3(1-bc(0))-(2*sf_[1].pts_[1])/POW2(1-bc(0))+(2*bc(1)*sf_[1].pts_[0])/POW3(1-bc(0)))-(2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))+(2*sf_[1].pts_[1])/(1-bc(0))-(2*bc(1)*sf_[1].pts_[0])/POW2(1-bc(0)))-2*bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0)))+2*pts_[ind[0]]).head<3>();
        M.col(1) = (2*pts_[ind[2]]+2*pts_[ind[1]]+bc(0)*((-bc(1)-bc(0)+1)*((2*bc(0)*sf_[2].pts_[1])/POW3(1-bc(1))+(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW3(1-bc(1))-(2*sf_[2].pts_[0])/POW2(1-bc(1)))-(2*bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))-(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))+(2*sf_[2].pts_[0])/(1-bc(1)))+(-bc(1)-bc(0)+1)*(bc(0)*((2*bc(0)*sf_[2].pts_[1])/POW3(1-bc(1))+(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW3(1-bc(1))-(2*sf_[2].pts_[0])/POW2(1-bc(1)))-(2*sf_[1].pts_[1])/(1-bc(0))+(2*sf_[1].pts_[0])/(1-bc(0)))-2*bc(0)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+bc(1)*((2*sf_[1].pts_[1])/(1-bc(0))-(2*sf_[1].pts_[0])/(1-bc(0)))-2*bc(1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))+2*(-bc(1)-bc(0)+1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))-(4*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(4*bc(1)*sf_[1].pts_[0])/(1-bc(0))).head<3>();

        //std::cout<<"C\n"<<M<<"\n";
      }
      else {
        M.col(0) = (2*pts_[ind[2]]+(-bc(1)-bc(0)+1)*((2*sf_[2].pts_[1])/(1-bc(1))-(2*sf_[2].pts_[0])/(1-bc(1))+bc(1)*((2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW3(1-bc(0))-(2*sf_[1].pts_[1])/POW2(1-bc(0))+(2*bc(1)*sf_[1].pts_[0])/POW3(1-bc(0))))+2*(-bc(1)-bc(0)+1)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))-2*bc(0)*(sf_[2].pts_[1]/(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+bc(0)*(-(2*sf_[2].pts_[1])/(1-bc(1))+(2*sf_[2].pts_[0])/(1-bc(1))+bc(1)*((2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))-(2*sf_[0].pts_[0])/POW2(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0))))-(4*bc(0)*sf_[2].pts_[1])/(1-bc(1))-
            (4*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/(1-bc(1))+bc(1)*((-bc(1)-bc(0)+1)*((2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW3(1-bc(0))-(2*sf_[1].pts_[1])/POW2(1-bc(0))+(2*bc(1)*sf_[1].pts_[0])/POW3(1-bc(0)))-(2*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))+(2*sf_[1].pts_[1])/(1-bc(0))-(2*bc(1)*sf_[1].pts_[0])/POW2(1-bc(0))+bc(0)*((2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))-(2*sf_[0].pts_[0])/POW2(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0)))-(2*bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*sf_[0].pts_[0])/(bc(1)+bc(0))-(2*bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))-2*bc(1)*(((-bc(1)-bc(0)+1)*sf_[1].pts_[1])/POW2(1-bc(0))-sf_[1].pts_[1]/(1-bc(0))+(bc(1)*sf_[1].pts_[0])/POW2(1-bc(0)))+2*bc(1)*
            (-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+sf_[0].pts_[0]/(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+2*pts_[ind[0]] ).head<3>();

        M.col(1) =
            (2*pts_[ind[2]]+2*pts_[ind[1]]+bc(0)*((-bc(1)-bc(0)+1)*((2*bc(0)*sf_[2].pts_[1])/POW3(1-bc(1))+(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW3(1-bc(1))-(2*sf_[2].pts_[0])/POW2(1-bc(1)))-(2*bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))-(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))+(2*sf_[2].pts_[0])/(1-bc(1))+bc(1)*(-(2*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0)))+(2*sf_[0].pts_[1])/(bc(1)+bc(0))-(2*bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(2*bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0)))+(-bc(1)-bc(0)+1)*
                (bc(0)*((2*bc(0)*sf_[2].pts_[1])/POW3(1-bc(1))+(2*(-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW3(1-bc(1))-(2*sf_[2].pts_[0])/POW2(1-bc(1)))-(2*sf_[1].pts_[1])/(1-bc(0))+(2*sf_[1].pts_[0])/(1-bc(0)))-2*bc(0)*((bc(0)*sf_[2].pts_[1])/POW2(1-bc(1))+((-bc(1)-bc(0)+1)*sf_[2].pts_[0])/POW2(1-bc(1))-sf_[2].pts_[0]/(1-bc(1)))+bc(1)*((2*sf_[1].pts_[1])/(1-bc(0))-(2*sf_[1].pts_[0])/(1-bc(0))+bc(0)*(-(2*sf_[0].pts_[1])/POW2(bc(1)+bc(0))+(2*bc(1)*sf_[0].pts_[1])/POW3(bc(1)+bc(0))+(2*bc(0)*sf_[0].pts_[0])/POW3(bc(1)+bc(0))))-2*bc(1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))+2*(-bc(1)-bc(0)+1)*(sf_[1].pts_[0]/(1-bc(0))-sf_[1].pts_[1]/(1-bc(0)))-
                (4*(-bc(1)-bc(0)+1)*sf_[1].pts_[1])/(1-bc(0))-(4*bc(1)*sf_[1].pts_[0])/(1-bc(0))+2*bc(0)*(sf_[0].pts_[1]/(bc(1)+bc(0))-(bc(1)*sf_[0].pts_[1])/POW2(bc(1)+bc(0))-(bc(0)*sf_[0].pts_[0])/POW2(bc(1)+bc(0))) ).head<3>();
      }

      if(!pcl_isfinite(M.sum())) {
        std::cout<<"INF3 v\n"<<M<<"\n";
        std::cout<<"bc\n"<<bc<<"\n";
        std::cout<<"T\n"<<_T_<<"\n";
        size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

        for(int i=0; i<3; i++) {
          std::cout<<"cp\n"<<pts_[ind[i]]<<"\n";
          std::cout<<"uv\n"<<uv_[i]<<"\n";
        }
      }

      return M;
    }

    inline bool inside(const Eigen::Vector2f &uv) const {
      Eigen::Vector2f br;

      br = _T_*(uv-uv_[2]);

      return ( std::abs(br(0))+std::abs(br(1))+std::abs(1-br(0)-br(1))<=1 );

    }

    inline bool complete_inside(const Eigen::Vector2f &uv) const {
      Eigen::Vector2f br;

      br = _T_*(uv-uv_[2]);

      return ( std::abs(br(0))+std::abs(br(1))+std::abs(1-br(0)-br(1))<1 );

    }

    static float sqDistLinePt(const Eigen::Vector2f &uv, const Eigen::Vector2f &r1, const Eigen::Vector2f &r2) {
      float f = (uv-r1).dot(r2-r1) / (r2-r1).squaredNorm();
      if(f>1) f=1;
      else if(f<0) f=0;
      return (uv - f*(r2-r1) - r1).squaredNorm();
    }

    bool polating(const Eigen::Vector3f &pt, const Eigen::Vector2f &uv, bool &inside, Eigen::Vector2f &r1, Eigen::Vector2f &r2) {
      static const size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };
      float dist[3] = { (pt-pts_[ind[0]].head<3>()).squaredNorm(), (pt-pts_[ind[1]].head<3>()).squaredNorm(), (pt-pts_[ind[2]].head<3>()).squaredNorm() };

      Eigen::Vector2f br;

      br = _T_*(uv-uv_[2]);
//      std::cout<<"br\n"<<br<<"\n";
//      std::cout<<"pt\n"<<pt<<"\n";

      if( std::abs(br(0))+std::abs(br(1))+std::abs(1-br(0)-br(1))<=1 ) {
        inside=true;
        r1=uv;
        return true;
        for(int i=0; i<3; i++) dist[i]=std::sqrt(dist[i]);
        br(0) = dist[0]/(dist[0]+dist[1]+dist[2]);
        br(1) = dist[1]/(dist[0]+dist[1]+dist[2]);
//        std::cout<<"uv3434\n"<<_T_.inverse()*br + uv_[2]<<"\n"<<br<<"\n"<<dist[2]/(dist[0]+dist[1]+dist[2])<<std::endl;
        r1 = _T_.inverse()*br + uv_[2];
        return true;
      }

      inside=false;

      int m=0;
      if(sqDistLinePt(uv,uv_[(1+1)%3],uv_[(1+2)%3])<sqDistLinePt(uv,uv_[(m+1)%3],uv_[(m+2)%3])) m=1;
      if(sqDistLinePt(uv,uv_[(2+1)%3],uv_[(2+2)%3])<sqDistLinePt(uv,uv_[(m+1)%3],uv_[(m+2)%3])) m=2;
      const int a = (m+1)%3;
      const int b = (m+2)%3;
      const float f = (uv_[a]-uv_[b]).squaredNorm()/(pts_[ind[a]]-pts_[ind[b]]).squaredNorm();

//      std::cout<<"r1 "<<std::sqrt(dist[a])<<"\n";
//      std::cout<<"r2 "<<std::sqrt(dist[b])<<"\n";
//      std::cout<<"r3 "<<(pts_[ind[a]]-pts_[ind[b]]).norm()<<"\n";

      Eigen::Vector2f p1, p2;

      if(pcl_isfinite(f)) {
        dist[a]*=f;
        dist[b]*=f;

        if(std::abs((2*uv_[b](0)-2*uv_[a](0)))>std::abs((2*uv_[b](1)-2*uv_[a](1)))) {
          const float n = -(2*uv_[b](1)-2*uv_[a](1)) / (2*uv_[b](0)-2*uv_[a](0));
          const float o = uv_[a](0)+(dist[b]-dist[a]+uv_[a].squaredNorm()-uv_[b].squaredNorm())/(2*uv_[b](0)-2*uv_[a](0));

          p1(1)=-(std::sqrt((n*n+1)*dist[a]-o*o+2*uv_[a](1)*n*o-uv_[a](1)*uv_[a](1)*n*n)-(n*o+uv_[a](1)))/(n*n+1);
          p2(1)= (std::sqrt((n*n+1)*dist[a]-o*o+2*uv_[a](1)*n*o-uv_[a](1)*uv_[a](1)*n*n)+(n*o+uv_[a](1)))/(n*n+1);
          p1(0) = n*p1(1) - o+uv_[a](0);
          p2(0) = n*p2(1) - o+uv_[a](0);

//          std::cout<<"n "<<n<<"\n";
//          std::cout<<"o "<<o<<std::endl;
        }
        else {
          const float n = -(2*uv_[b](0)-2*uv_[a](0)) / (2*uv_[b](1)-2*uv_[a](1));
          const float o = uv_[a](1)+(dist[b]-dist[a]+uv_[a].squaredNorm()-uv_[b].squaredNorm())/(2*uv_[b](1)-2*uv_[a](1));

          p1(0)=-(std::sqrt((n*n+1)*dist[a]-o*o+2*uv_[a](0)*n*o-uv_[a](0)*uv_[a](0)*n*n)-(n*o+uv_[a](0)))/(n*n+1);
          p2(0)= (std::sqrt((n*n+1)*dist[a]-o*o+2*uv_[a](0)*n*o-uv_[a](0)*uv_[a](0)*n*n)+(n*o+uv_[a](0)))/(n*n+1);
          p1(1) = n*p1(0) - o+uv_[a](1);
          p2(1) = n*p2(0) - o+uv_[a](1);

//          std::cout<<"n "<<n<<"\n";
//          std::cout<<"o "<<o<<std::endl;
        }
      }

//      std::cout<<"a\n"<<uv_[a]<<"\n";
//      std::cout<<"b\n"<<uv_[b]<<"\n";
//      std::cout<<"r1 "<<std::sqrt(dist[a])<<"\n";
//      std::cout<<"r2 "<<std::sqrt(dist[b])<<"\n";
//      std::cout<<"r3 "<<(uv_[a]-uv_[b]).norm()<<"\n";
//      std::cout<<"r\n"<<(uv_[a]-uv_[b]).squaredNorm()<<"\n";
//      std::cout<<"p1\n"<<p1<<"\n";
//      std::cout<<"p2\n"<<p2<<std::endl;
//      std::cout<<"uvm\n"<<uv_[m]<<std::endl;
//      std::cout<<"d1 "<<(p1-uv_[m]).squaredNorm()<<"\n";
//      std::cout<<"d2 "<<(p2-uv_[m]).squaredNorm()<<std::endl;

      if(!pcl_isfinite(f)|| (!pcl_isfinite(p1.sum()) && !pcl_isfinite(p2.sum())) ) {
        dist[a] = std::sqrt(dist[a]);
        dist[b] = std::sqrt(dist[b]);

//        std::cout<<"using second\n"<<std::endl;

        r1 = (uv_[a]*dist[a] + uv_[b]*dist[b])/(dist[a]+dist[b]);
        inside=true;
        return true;
      }

//      std::cout<<"br\n"<< _T_*(uv-uv_[2])<<std::endl;
//      ROS_ASSERT( !(this->inside(p1)&&this->inside(p2)) );

      r1 = p1;
      r2 = p2;
      return false;
    }

    void print() const {
      static const size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };
      for(int i=0; i<3; i++) {
        std::cout<<"pt\n"<<pts_[ind[i]]<<"\n";
        std::cout<<"n\n"<<n_[ind[i]]<<"\n";
        std::cout<<"n2\n"<<n2_[ind[i]]<<"\n";
      }
      for(int i=0; i<3; i++)
        sf_[i].print();
      std::cout<<std::endl;
    }

  };
}

#endif /* TRISPLINE_H_ */
