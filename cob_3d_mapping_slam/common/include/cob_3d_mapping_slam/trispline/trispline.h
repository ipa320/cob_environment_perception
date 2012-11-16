/*
 * trispline.h
 *
 *  Created on: 25.10.2012
 *      Author: josh
 */

#ifndef TRISPLINE_H_
#define TRISPLINE_H_

#include <Eigen/Core>

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

    inline Eigen::Vector3f operator()(const Eigen::Vector3f &bc, const Eigen::Vector3f* pts_) const {
      return tri_a_(bc, pts_)*bc( (0+Round)%3 ) + tri_b_(bc, pts_)*bc( (1+Round)%3 ) + tri_c_(bc, pts_)*bc( (2+Round)%3 );
    }

    inline Eigen::Vector3f normalAt(const Eigen::Vector3f &bc, const Eigen::Vector3f* pts_) const {
      const Eigen::Vector3f a = tri_a_(bc, pts_), b=tri_b_(bc, pts_), c=tri_c_(bc, pts_);
      return (b-a).cross(c-a);
    }

    inline Eigen::Vector3f normalAt2(const Eigen::Vector3f &bc, const Eigen::Vector3f* pts_) const {
      const Eigen::Vector3f a = normalAt(bc, pts_), b=normalAt(bc, pts_), c=normalAt(bc, pts_);
      return (b-a).cross(c-a);
    }

    inline Eigen::Matrix4f normalAtUV(const Eigen::Vector3f &bc, const Eigen::Vector3f* pts_, const Eigen::Vector2f *_uv) const {
      const Eigen::Vector3f a = tri_a_(bc, pts_), b=tri_b_(bc, pts_), c=tri_c_(bc, pts_);
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
        const Eigen::Vector3f a = pts_[Ca::ind], b=pts_[Cb::ind], c=pts_[Cc::ind];
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

    void print(const Eigen::Vector3f* pts_) const {
      tri_a_.print(pts_);
      tri_b_.print(pts_);
      tri_c_.print(pts_);
    }

  };

  template<typename Ca, typename Cb, typename Cc, int Round>
  class TriangleC<Ca,Cb,Cc,Round,0> {
  public:

    inline Eigen::Vector3f operator()(const Eigen::Vector3f &bc, const Eigen::Vector3f* pts_) const {
      return pts_[Ca::ind]*bc( (0+Round)%3 ) + pts_[Cb::ind]*bc( (1+Round)%3 ) + pts_[Cc::ind]*bc( (2+Round)%3 );
    }

    inline Eigen::Vector3f normalAt(const Eigen::Vector3f &bc, const Eigen::Vector3f* pts_) const {
      const Eigen::Vector3f a = pts_[Ca::ind], b=pts_[Cb::ind], c=pts_[Cc::ind];
      return (b-a).cross(c-a);
    }


    void print(const Eigen::Vector3f* pts_) const {
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

    inline Eigen::Vector3f intersection(const _Line &l) {
#if DEBUG_OUT_
      std::cout<<"np\n"<<n<<"\n";
#endif
      const float d=l.u.dot(n);
      if( std::abs(d)< 0.0001f )
        return 0.5f*(o+l.o);
      return l.at( (o-l.o).dot(n)/d );
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
    Eigen::Vector3f pts_[2];
  public:

    inline Eigen::Vector3f delta() const {return pts_[0]-pts_[1];}

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
               const Eigen::Vector3f &na2, const Eigen::Vector3f &nb2)
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

      pts_[0] = p1.intersection(_Line(na2,0.5f*(a+b)));
      pts_[1] = p1.intersection(_Line(nb2,0.5f*(a+b)));

#endif

#endif

#if DEBUG_OUT_
      print();
#endif
    }

    void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {
      pts_[0] = rot*pts_[0]+tr;
      pts_[1] = rot*pts_[1]+tr;
    }

    inline Eigen::Vector3f operator()(const float a, const float b) {
      float f = a/(a+b);
      if(!pcl_isfinite(f)) f=0.f;
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

    Eigen::Vector3f pts_[PT_SIZE];

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
      return tri_(bc, pts_);
    }

    virtual Eigen::Vector3f normalAt(const Eigen::Vector3f &bc) const {
      return tri_.normalAt(bc, pts_);
    }

    virtual Eigen::Vector3f normalAt2(const Eigen::Vector3f &bc) const {
      return tri_.normalAt2(bc, pts_);
    }

    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {
      for(int i=0; i<PT_SIZE; i++)
        pts_[i] = rot*pts_[i]+tr;
    }

  };

  class TriSpline2_Fade : public TriSpline<2> {
    Eigen::Vector3f n_[3], n2_[3];
    Eigen::Vector2f uv_[3];
    SplineFade sf_[3];
    Eigen::Matrix2f _T_;

    Eigen::Vector3f m_;

    void setup() {
      size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

      for(int i=0; i<3; i++) {
        sf_[i].setup( pts_[ind[i]], pts_[ind[(i+1)%3]],
                      n_[i], n_[(i+1)%3],
                      n2_[i], n2_[(i+1)%3]);
        m_ += pts_[ind[i]];
      }

      m_/=3;
    }

  public:

    void test_setup(_Line *l) {
      size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };

      for(int i=0; i<3; i++) {
        sf_[i].test_setup( pts_[ind[i]], pts_[ind[(i+1)%3]],
                      n_[i], n_[(i+1)%3],
                      n2_[i], n2_[(i+1)%3],
                      l[i*2+0],l[i*2+1]);
      }
    }

    inline Eigen::Vector3f getEdge(const int i) {
      static const size_t ind[3] = { indAB(0,ORDER), indAB(ORDER,ORDER), indAB(0,0) };
      return pts_[ind[i]];
    }

    inline SplineFade getFade(const int i) {return sf_[i];}
    inline Eigen::Vector3f getNormal(const int i) {return n_[i];}
    inline Eigen::Vector2f getUV(const int i) {return uv_[i];}
    inline Eigen::Vector3f getNormal2(const int i) {return n2_[i];}

    TriSpline2_Fade(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c,
                    const Eigen::Vector3f &na, const Eigen::Vector3f &nb, const Eigen::Vector3f &nc,
                    const Eigen::Vector3f &na2, const Eigen::Vector3f &nb2, const Eigen::Vector3f &nc2,
                    const Eigen::Vector2f &uva, const Eigen::Vector2f &uvb, const Eigen::Vector2f &uvc
    )
    {
      pts_[indAB(0,ORDER)] = a;
      pts_[indAB(ORDER,ORDER)] = b;
      pts_[indAB(0,0)] = c;

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

      setup();
    }

    virtual Eigen::Vector3f operator()(const Eigen::Vector3f &bc) {
      pts_[indAB(ORDER/2,ORDER)]	= sf_[0](bc(0),bc(1));
      pts_[indAB(ORDER/2,ORDER/2)]      = sf_[1](bc(1),bc(2));
      pts_[indAB(0,ORDER/2)]            = sf_[2](bc(2),bc(0));
#if DEBUG_OUT_
      tri_.print(pts_);
#endif
      return tri_(bc, pts_);
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

      return (*this)(br);
    }

    Eigen::Vector3f normalAt(const Eigen::Vector2f &pt) {
      //1. bayrcentric coordinates 2D -> 3D
      Eigen::Vector3f br;
      br.head<2>() = _T_*(pt-uv_[2]);
      br(2) = 1-br(0)-br(1);

      return normalAt(br);
    }

    Eigen::Vector3f normalAt2(const Eigen::Vector2f &pt) {
      //1. bayrcentric coordinates 2D -> 3D
      Eigen::Vector3f br;
      br.head<2>() = _T_*(pt-uv_[2]);
      br(2) = 1-br(0)-br(1);

      return normalAt2(br);
    }

    virtual void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {
      TriSpline<2>::transform(rot, tr);
      for(int i=0; i<3; i++) {
        std::cout<<tr;
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

  };
}

#endif /* TRISPLINE_H_ */
