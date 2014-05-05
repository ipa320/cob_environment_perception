/*
 * trispline.h
 *
 *  Created on: 25.10.2012
 *      Author: josh
 */

#ifndef TOPOLGY_H_
#define TOPOLGY_H_

#define CGAL_DISABLE_ROUNDING_MATH_CHECK

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Homogeneous_d.h>
#include <CGAL/leda_integer.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <unsupported/Eigen/NonLinearOptimization>
#include <eigen3/Eigen/Jacobi>

#include <cob_3d_mapping_slam/marker/marker_container.h>

namespace ParametricSurface {

  class Topology
  {
  public:

    class POINT;

  private:

#if 0
    /* A vertex class with an additionnal handle */
    template < class Gt, class Vb = CGAL::Triangulation_vertex_base_2<Gt> >
    class Vertex_base_with_ptr
    : public  Vb
      {
      typedef Vb                              Base;
      public:
      typedef typename Vb::Vertex_handle      Vertex_handle;
      typedef typename Vb::Face_handle        Face_handle;
      typedef typename Vb::Point              Point;

      template < typename TDS2 >
      struct Rebind_TDS {
        typedef typename Vb::template Rebind_TDS<TDS2>::Other    Vb2;
        typedef My_vertex_base<Gt,Vb2>                           Other;
      };

      private:
      boost::shared_ptr<POINT>  va_;

      public:
      Vertex_base_with_ptr() : Base() {}
      Vertex_base_with_ptr(const Point & p) : Base(p) {}
      Vertex_base_with_ptr(const Point & p, Face_handle f) : Base(f,p) {}
      Vertex_base_with_ptr(Face_handle f) : Base(f) {}

      void set_associated_point(const boost::shared_ptr<POINT> &va) { va_ = va;}
      boost::shared_ptr<POINT> get_associated_point() {return va_ ; }
      };
#endif
    //typedef leda_integer RT;
    typedef float RT;
    //typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef CGAL::Cartesian<float>                               Kernel;
    //typedef CGAL::Homogeneous_d<double>                               Kernel;

    typedef CGAL::Triangulation_vertex_base_with_info_2<boost::shared_ptr<POINT>, Kernel>    Vb;
    typedef CGAL::Triangulation_face_base_with_info_2<boost::shared_ptr<ParametricSurface::TriSpline2_Fade>, Kernel>    Vbb;
    typedef CGAL::Triangulation_data_structure_2<Vb, Vbb>                    Tds;

    typedef CGAL::Delaunay_triangulation_2<Kernel, Tds> Delaunay_d;
    typedef Delaunay_d::Point Point;
    typedef Kernel::Vector_2 Vector;
    typedef Delaunay_d::Vertex Vertex;
    typedef Delaunay_d::Face Face;
    typedef Delaunay_d::Face_handle Face_handle;
    typedef Delaunay_d::Face_iterator Face_iterator;
    typedef Delaunay_d::All_edges_iterator Edge_iterator;

    //typedef Delaunay_d::Simplex_handle Simplex_handle;
    //typedef Delaunay_d::Simplex_const_iterator Simplex_const_iterator;
    //typedef Delaunay_d::Point_const_iterator Point_const_iterator;
    //typedef Delaunay_d::Simplex_iterator Simplex_iterator;
    typedef Delaunay_d::Vertex_handle Vertex_handle;
    typedef Delaunay_d::Face_circulator Face_circulator;
    typedef Delaunay_d::Vertex_circulator Vertex_circulator;
    typedef Delaunay_d::Vertex_iterator Vertex_iterator;

    public:

    struct  POINT {
      Eigen::Vector2f uv;
      Eigen::Vector3f pt, n, n2;
      Vertex_handle vh;
      float weight_;

      POINT(): weight_(1) {}

      void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {
        pt = rot*pt + tr;
        n  = rot*n;
        n2 = rot*n2;
      }
    };


    private:

    Delaunay_d del_;
    std::vector< boost::shared_ptr<POINT> > pts_;
    std::vector< boost::shared_ptr<ParametricSurface::TriSpline2_Fade> > tris_;
    //    std::map< Vertex*, boost::shared_ptr<POINT> > map_pts_;
    //    std::map< Face*, boost::shared_ptr<ParametricSurface::TriSpline2_Fade> > map_tris_;
    float thr_;

    bool isOnBorder(const boost::shared_ptr<POINT> &pt) const {
      if(pts_.size()<=3)
        return true;
#if 0
      //1. if its part of border -> don't delete (for now)
      //      std::cout<<"addr1 "<<pt.get()<<"\n";
      //      std::cout<<"addr2 "<<&*pt->vh<<"\n";
      //Face_handle fh = pt->vh->face();
      Face_circulator fc = pt->vh->incident_faces();
      if (fc != 0) {
        int n=0;
        bool b1=false, b2=false;
        Vector p;
        do {
          Vertex_handle v1 = fc->vertex(0), v2 = fc->vertex(1);
          if(v1==pt->vh) v1 = fc->vertex(2);
          if(v2==pt->vh) v2 = fc->vertex(2);

          //          std::cout<<"p1\n"<<v1->point()<<"\n";
          //          std::cout<<"p2\n"<<v2->point()<<"\n";

          if(!n) {
            p = v1->point()-pt->vh->point();
          }
          float f;
          Vector v;

          v = v1->point()-pt->vh->point();
          if( v.x()*p.x()+v.y()*p.y()<=0 ) {
            f = v.x()*p.y()-v.y()*p.x();
            if(f>0) b1=true; else if(f<0) b2=true;

            //            std::cout<<"b "<<b1<<" "<<b2<<"\n";
            //            std::cout<<"f2 "<<f<<"\n";
          }

          v = v2->point()-pt->vh->point();
          if( v.x()*p.x()+v.y()*p.y()<=0 ) {
            f = v.x()*p.y()-v.y()*p.x();
            if(f>0) b1=true; else if(f<0) b2=true;

            //            std::cout<<"b "<<b1<<" "<<b2<<"\n";
            //            std::cout<<"f2 "<<f<<"\n";
          }

          if(b1&&b2) break;
          ++n;
        } while (++fc != pt->vh->incident_faces());

        if(!b1||!b2) return true;
      }
      else
        return true;
#else
      Vertex_circulator start = del_.incident_vertices(del_.infinite_vertex());
      Vertex_circulator vc(start);
      do {
//        std::cout<<"border check "<<(pt->vh->point()==vc->point())<<"\n";
        if(pt->vh->point()==vc->point())
          return true;
      } while( (++vc)!=start);
#endif
      return false;
    }

    bool canRemove(const boost::shared_ptr<POINT> &pt) const {
      //2. get PCA of \delta of each edge connecting vertex
      //   -> if smallest eigen value less than threshold
      //   -> delete
      Eigen::Matrix3f V = Eigen::Matrix3f::Zero();

      Face_circulator fc = pt->vh->incident_faces();
      float n=0;
      do {
        //ROS_ASSERT(map_tris_.find(&*fc)!=map_tris_.end());

        if( fc->info() ) { //TODO: check why this is needed
          Eigen::Matrix3f delta = fc->info()->delta();
          for(int i=0; i<3; i++)
            V+=delta.col(i)*delta.col(i).transpose() / (fc->info()->getEdge(i)-fc->info()->getEdge((i+1)%3)).squaredNorm();
          n+=3.f;
        }
      } while (++fc != pt->vh->incident_faces());

      Eigen::JacobiSVD<Eigen::Matrix3f> svd (V, Eigen::ComputeFullU | Eigen::ComputeFullV);
//      std::cout<<"SVD\n"<<svd.singularValues()<<"\n";

      return
          //std::min(std::min(std::abs(svd.singularValues()(0)),std::abs(svd.singularValues()(1))),std::abs(svd.singularValues()(2)))
          std::abs(svd.singularValues()(2))
      <thr_*n;
    }

    void removePoint(const size_t i) {
      //map_pts_.erase(&*pts_[i]->vh);
      ROS_ASSERT(pts_[i]->vh->info()==*(pts_.begin()+i));
//      ROS_ASSERT( !del_.test_dim_down(pts_[i]->vh) );
      del_.remove(pts_[i]->vh);

//      std::list<Edge> hole;
//      del_.make_hole(v, hole);
//      del_.fill_hole_delaunay(hole);
//      del_.delete_vertex(v);

      pts_.erase(pts_.begin()+i);
    }

    // for LM
    struct Functor {
      Eigen::Vector3f pt_;
      const Topology &top_;
      ParametricSurface::TriSpline2_Fade *last_;

      Functor(const Topology &t, const Eigen::Vector3f &pt):pt_(pt), top_(t), last_(NULL)
      {}

      ParametricSurface::TriSpline2_Fade *getLast() const {return last_;}

      void init(Eigen::VectorXf &x) {
        last_ = top_.locate(x);

        Eigen::Vector3f delta = ((*last_)((Eigen::Vector2f)x)-pt_);
        Eigen::Matrix<float,3,2> M = last_->normal(x);

//        std::cout<<"start\n"<<x<<"\n"<<M<<"\n";
//        std::cout<<"a "<<-M.col(0).dot( delta )<<"\n";
//        std::cout<<"b "<<-M.col(1).dot( delta )<<"\n";

        Eigen::VectorXf x2 = x;
        x2(0) -= M.col(0).dot( delta );
        x2(1) -= M.col(1).dot( delta );

        if(last_->inside(x2))
          x = x2;
      }

      int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
      {
        // distance
        if(!last_->inside(x))
          ((Functor*)this)->last_ = top_.locate(x);
        ParametricSurface::TriSpline2_Fade *l = top_.locate(x);
        fvec(0) = (pt_ - (*l)((Eigen::Vector2f)x)).squaredNorm();
        fvec(1) = 0;

//        std::cout<<"x\n"<<x<<"\n";
//        std::cout<<"dist "<<fvec(0)<<"\n";
        return 0;
      }

      int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
      {
        //Jacobian
#if 0
        Eigen::Matrix4f p = top_.normalAtUV(x);

        Eigen::Vector3f vx, vy;
        vx = p.col(3).head<3>().cross(p.col(1).head<3>()).cross(p.col(3).head<3>());
        vy = p.col(3).head<3>().cross(p.col(2).head<3>()).cross(p.col(3).head<3>());

        vx.normalize();
        vy.normalize();

        Eigen::Matrix3f M;
        M.col(0) = vx;
        M.col(1) = -vy;
        M.col(2) = p.col(3).head<3>();

        fjac.row(0) = 2*(M.inverse()*(p.col(0).head<3>()-pt_)).head<2>();

        std::cout<<"o1 "<<vx.dot(p.col(0).head<3>()-pt_)<<"\n";
        std::cout<<"o2 "<<vx.dot(p.col(1).head<3>()-pt_)<<"\n";
        std::cout<<"p\n"<<p<<"\n";
#else
        if(!last_->inside(x))
          ((Functor*)this)->last_ = top_.locate(x);
        ParametricSurface::TriSpline2_Fade *l = top_.locate(x);
        Eigen::Vector3f v = (*l)((Eigen::Vector2f)x)-pt_;
        Eigen::Matrix<float,3,2> M = l->normal(x);

        fjac(0,0) = 2*M.col(0).dot( v );
        fjac(0,1) = 2*M.col(1).dot( v );
#endif

        //        fjac(0,0) = 2*( p.col(1)(3)* p.col(3).head<3>().dot(p.col(0).head<3>()-pt_) + vx.dot(p.col(0).head<3>()-pt_));
        //        fjac(0,1) = -2*( p.col(2)(3)* p.col(3).head<3>().dot(p.col(0).head<3>()-pt_) + vy.dot(p.col(0).head<3>()-pt_));
        fjac(1,0) = fjac(1,1) = 0;

//        std::cout<<"x\n"<<x<<"\n";
//        std::cout<<"fjac\n"<<fjac<<"\n";
//        std::cout<<"d\n"<<M<<"\n";

        return 0;
      }

      int inputs() const { return 2; }
      int values() const { return 2; } // number of constraints
    };

    friend class Functor;

    static float sqDistLinePt(const Eigen::Vector2f &uv, const Eigen::Vector2f &r1, const Eigen::Vector2f &r2) {
      float f = (uv-r1).dot(r2-r1) / (r2-r1).squaredNorm();
      if(f>1) f=1;
      else if(f<0) f=0;
      return (uv - f*(r2-r1) - r1).squaredNorm();
    }

    inline ParametricSurface::TriSpline2_Fade* locate(const Eigen::Vector2f &uv) const {
      bool inside;
      return locate(uv,inside);
    }
    inline ParametricSurface::TriSpline2_Fade* locate(const Eigen::Vector2f &uv, bool &inside) const {
      Face_handle sh = del_.locate( Point(uv(0),uv(1)) );

      /*if(sh==del_.faces_end()) {
        ROS_WARN("invalid pt requested");
        return tris_[0].get();
      }*/

      if(!sh->info().get()) {
        ParametricSurface::TriSpline2_Fade* r=NULL;
        inside=false;

        float mi = std::numeric_limits<float>::max();
        for(std::vector< boost::shared_ptr<ParametricSurface::TriSpline2_Fade> >::const_iterator it = tris_.begin();
            it!=tris_.end(); ++it)
        {

          Eigen::Vector2f p1 = (*it)->getUV(0),p2=(*it)->getUV(1),p3=(*it)->getUV(2);

          //const float A = ((*it)->getEdge(1)-(*it)->getEdge(0)).cross((*it)->getEdge(2)-(*it)->getEdge(0)).squaredNorm();

          const float dist = std::min( sqDistLinePt(uv, p1,p2), std::min(sqDistLinePt(uv, p2,p3), sqDistLinePt(uv, p3,p1)));

          //std::cout<<"dist "<<dist<<"\n";
          if(dist<mi) {
            mi = dist;
            r = it->get();
          }

        }

        //std::cout<<"tri size "<<tris_.size()<<std::endl;

        ROS_ASSERT(r);
        return r;
      }
      else {
        //ROS_ASSERT(sh->info().get());
        inside=true;
        return sh->info().get();
      }

    }

    public:

    Topology(const float thr): thr_(thr)
    {
    }

    Topology(const Topology &o) {
      *this = o;
    }

    void operator=(const Topology &o) {
      thr_ = o.thr_;
#if 0
      del_ = o.del_;

      for(size_t i=0; i<o.pts_.size(); i++) {
        pts_.push_back( boost::shared_ptr<POINT>( new POINT(*o.pts_[i])) );

        pts_.back()->vh = del_.nearest_vertex( Point(pts_.back()->uv(0), pts_.back()->uv(1)) );
        pts_.back()->vh->info() = pts_[i];
      }
#else
      for(size_t i=0; i<o.pts_.size(); i++)
        insertPointWithoutUpdate( *o.pts_[i] );
#endif

      update();
    }

    void insertPoint(const POINT &pt) {
      insertPointWithoutUpdate(pt);
      update();
    }

    void finish() {
//      std::cout<<"FINISH----------\n";
      std::vector<bool> border(pts_.size());
      for(size_t i=0; i<pts_.size(); i++)
        border[i] = isOnBorder(pts_[i]);

      for(size_t i=0; /*pts_.size()>3 &&*/ i<pts_.size(); i++) {
        if( !border[i] && canRemove(pts_[i]) ) {
//          std::cout<<"remove point "<<i<<"\n"<<pts_[i]->uv<<"\n";
//          ROS_ASSERT(pts_.size()>3);
          border.erase(border.begin()+i);
          removePoint(i);
          update();
          --i;
        }
      }

//      for(Vertex_iterator it = del_.vertices_begin(); it!=del_.vertices_end(); it++) {
//        //ROS_ASSERT( it->info()->vh == it);
//        ROS_ASSERT(it->point().x()==it->info()->uv(0));
//        ROS_ASSERT(it->point().y()==it->info()->uv(1));
//      }
//
//      for(size_t i=0; i<pts_.size(); i++) {
////        std::cout<<pts_[i]->uv<<"\n";
////        std::cout<<pts_[i]->vh->point().x()<<" "<<pts_[i]->vh->point().y()<<"\n"<<std::endl;
//        ROS_ASSERT(pts_[i]->vh->point().x()==pts_[i]->uv(0));
//        ROS_ASSERT(pts_[i]->vh->point().y()==pts_[i]->uv(1));
//      }

      bool found = true;
      while(found && pts_.size()>3) {
        found = false;
//        std::cout<<tris_.size()<<"\n";
//        std::cout<<del_.number_of_faces()<<"\n";
        for(Edge_iterator ei=del_.all_edges_begin(); ei!=del_.all_edges_end(); ei++) {
          // Get a vertex from the edge
          Face_handle f = (ei->first);
          int i = ei->second;
          if(!f->info()) continue;
          Vertex_handle vs = f->vertex(f->cw(i));
          Vertex_handle vt = f->vertex(f->ccw(i));
          if(!vs->info() || !vt->info() || vs==vt || vs->point()==vt->point()) continue;
          if( (vt->info()->pt-vs->info()->pt).squaredNorm() < 0.03f*0.03f) {
//            std::cout<<"XXX "<<&*f<<" "<<f->info()<<" "<<i<<"\n";
//            ROS_INFO("merging uv pts");
//            std::cout<<vs->info()->uv<<"\n";
//            std::cout<<vt->info()->uv<<"\n";
//            std::cout<<vs->point().x()<<" "<<vs->point().y()<<"\n";
//            std::cout<<vt->point().x()<<" "<<vt->point().y()<<"\n";

            POINT pt;
            pt.uv = (vt->info()->uv*vt->info()->weight_ + vs->info()->uv*vs->info()->weight_)/(vt->info()->weight_+vs->info()->weight_);
            pt.pt = (vt->info()->pt*vt->info()->weight_ + vs->info()->pt*vs->info()->weight_)/(vt->info()->weight_+vs->info()->weight_);
            pt.n  = (vt->info()->n *vt->info()->weight_ + vs->info()->n *vs->info()->weight_)/(vt->info()->weight_+vs->info()->weight_);
            pt.n2 = (vt->info()->n2*vt->info()->weight_ + vs->info()->n2*vs->info()->weight_)/(vt->info()->weight_+vs->info()->weight_);

            for(size_t j=0; j<pts_.size(); j++)
              if(pts_[j]==vt->info()||pts_[j]==vs->info()) {
                removePoint(j);
                border.erase(border.begin()+j);
                --j;
                found = true;
//                std::cout<<"remove point2\n";
              }
            ROS_ASSERT(found);

            insertPoint(pt);

            break;
          }
        }
//        std::cout<<del_.number_of_faces()<<"\n";
      }

      found = true;
      while(found && pts_.size()>3) {
        found = false;
        for(size_t k=0; k<tris_.size(); k++) {
          if( (tris_[k]->getEdge(1)-tris_[k]->getEdge(0)).cross( tris_[k]->getEdge(2)-tris_[k]->getEdge(0) ).squaredNorm() < 0.02f*0.02f*0.02f) {
//            ROS_INFO("merging uv pts3 %f", (tris_[k]->getEdge(1)-tris_[k]->getEdge(0)).cross( tris_[k]->getEdge(2)-tris_[k]->getEdge(0) ).squaredNorm());

            size_t n=(size_t)-1;
            size_t m=(size_t)-1;
            for(size_t j=0; j<pts_.size(); j++)
              if(pts_[j]->uv==tris_[k]->getUV(0)) {
                n=j;
                break;
              }
//            if(n==-1) {
//              for(size_t j=0; j<pts_.size(); j++)
//                std::cout<<"uv\n"<<pts_[j]->uv<<"\n";
//              std::cout<<"searching\n"<<tris_[k]->getUV(0)<<std::endl;
//            }
            ROS_ASSERT(n!=(size_t)-1);
            for(size_t j=0; j<pts_.size(); j++)
              if(pts_[j]->uv==tris_[k]->getUV(1)) {
                m=j;
                break;
              }
//            if(m==-1) {
//              for(size_t j=0; j<pts_.size(); j++)
//                std::cout<<"uv\n"<<pts_[j]->uv<<"\n";
//              std::cout<<"searching\n"<<tris_[k]->getUV(1)<<std::endl;
//            }
            ROS_ASSERT(m!=(size_t)-1);

            if(border[m]||border[n])
              continue;

            found = true;
            POINT pt;
            pt.uv = (pts_[m]->uv*pts_[m]->weight_ + pts_[n]->uv*pts_[n]->weight_)/(pts_[m]->weight_+pts_[n]->weight_);
            pt.pt = (pts_[m]->pt*pts_[m]->weight_ + pts_[n]->pt*pts_[n]->weight_)/(pts_[m]->weight_+pts_[n]->weight_);
            pt.n  = (pts_[m]->n *pts_[m]->weight_ + pts_[n]->n *pts_[n]->weight_)/(pts_[m]->weight_+pts_[n]->weight_);
            pt.n2 = (pts_[m]->n2*pts_[m]->weight_ + pts_[n]->n2*pts_[n]->weight_)/(pts_[m]->weight_+pts_[n]->weight_);

            std::cout<<pt.uv<<std::endl;
            removePoint(m);
            border.erase(border.begin()+m);
            if(n>m) --n;
            removePoint(n);
            border.erase(border.begin()+n);
            insertPoint(pt);
            break;
          }
        }
//        std::cout<<del_.number_of_faces()<<"\n";
      }

    }

    void insertPointWithoutUpdate(const POINT &pt) {
      ROS_ASSERT( pcl_isfinite(pt.uv.sum()) );

      pts_.push_back(boost::shared_ptr<POINT>( new POINT(pt)));
      pts_.back()->vh = del_.insert(Point(pt.uv(0),pt.uv(1)));
      if( pts_.back()->vh->info() ) {
        ROS_WARN("pt (%f, %f) already there", pt.uv(0), pt.uv(1));
        pts_.erase(pts_.end()-1);
        return;
      }
      pts_.back()->vh->info() = pts_.back();
    }

    void update() {

      tris_.clear();
      for(Face_iterator it = del_.faces_begin(); it!=del_.faces_end(); it++) {
        //          std::cout<<"addr2 "<<&*it->vertex(0)<<"\n";
        //          POINT *a = map_pts_[&*it->vertex(0)].get();
        //          POINT *b = map_pts_[&*it->vertex(1)].get();
        //          POINT *c = map_pts_[&*it->vertex(2)].get();
        POINT *a = it->vertex(0)->info().get();
        POINT *b = it->vertex(1)->info().get();
        POINT *c = it->vertex(2)->info().get();
        Eigen::Vector3f w;
        w(0) = a->weight_;
        w(1) = b->weight_;
        w(2) = c->weight_;
        tris_.push_back( boost::shared_ptr<ParametricSurface::TriSpline2_Fade>(new ParametricSurface::TriSpline2_Fade(
            a->pt,      b->pt,  c->pt,
            a->n,       b->n,   c->n,
            a->n2,      b->n2,  c->n2,
            a->uv,      b->uv,  c->uv,
            w, 2.f
        ) ));

        it->info() = tris_.back();
        //it->pp = tris_.back().get();
        //map_tris_[&*it] = tris_.back();
      }

    }

    inline Eigen::Vector2f nextPoint(const Eigen::Vector3f &p, ParametricSurface::TriSpline2_Fade **used=NULL) const {
      //std::cout<<"nP\n"<<p<<"\n";
      Eigen::VectorXf r(2);

      r = p.head<2>();
      float dis = (project2world(r)-p).squaredNorm();
//      float dis = std::numeric_limits<float>::max();
      /*for(std::vector< boost::shared_ptr<ParametricSurface::TriSpline2_Fade> >::const_iterator it = tris_.begin();
          it!=tris_.end(); ++it)
      {
        const float d = ((*it)->getMid()-p).squaredNorm();
        if(d<dis) {
          dis = d;
          r = (*it)->getMidUV();
        }
      }*/
      for(size_t i=0; i<pts_.size(); i++)
      {
        const float d = (pts_[i]->pt-p).squaredNorm();
        //std::cout<<"wuv "<<d<<"\n"<<pts_[i]->uv<<"\n";
        if(d<dis) {
          dis = d;
          r = pts_[i]->uv;
        }
      }
      /*for(size_t i=0; i<tris_.size(); i++)
      {
        const float d = ((tris_[i]->getEdge(2)-p).squaredNorm()+(tris_[i]->getEdge(1)-p).squaredNorm()+(tris_[i]->getEdge(0)-p).squaredNorm()) /
        ((tris_[i]->getEdge(2)-tris_[i]->getEdge(1)).squaredNorm()+(tris_[i]->getEdge(1)-tris_[i]->getEdge(0)).squaredNorm()+(tris_[i]->getEdge(2)-tris_[i]->getEdge(0)).squaredNorm());
        //std::cout<<"wuv "<<d<<"\n"<<pts_[i]->uv<<"\n";
        if(d<dis) {
          dis = d;
          r = pts_[i]->uv;
        }
      }*/
      //std::cout<<"start\n"<<r<<"\n";
      Functor functor(*this, p);
      functor.init(r);
#if 0
      Eigen::HybridNonLinearSolver<Functor, float> lm(functor);
      lm.parameters.maxfev = 50; //not to often (performance)
      lm.solve(r);
#else
      Eigen::LevenbergMarquardt<Functor, float> lm(functor);
      lm.parameters.maxfev = 50; //not to often (performance)
//      std::cout<<"xtol "<<lm.parameters.xtol<<"\n";
//      std::cout<<"ftol "<<lm.parameters.ftol<<"\n";
      lm.parameters.xtol = 0.001f;
      lm.parameters.ftol = 0.001f;
      //lm.parameters.factor = 10;
      lm.minimize(r);


      /*Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeInit(r);
      do {
          status = lm.minimizeOneStep(r);
          std::cout<<"status "<<status<<"\n";
      } while (status==Eigen::LevenbergMarquardtSpace::Running);*/
#endif

      if(used)
        *used = functor.getLast();

      ROS_ASSERT( pcl_isfinite(r.sum()) );

      return r;
    }

    inline Eigen::Vector3f project2world(const Eigen::Vector2f &uv) const {
      if(tris_.size()<3) Eigen::Vector3f::Zero();
      Eigen::Vector3f r = (*locate(uv))(uv);

      if( !pcl_isfinite(r.sum()) ) {
        locate(uv)->print();
        std::cout<<"uv is\n"<<uv<<std::endl;
        std::cout<<"r is\n"<<r<<std::endl;
        ROS_ASSERT( pcl_isfinite(r.sum()) );
      }

      return r;
    }

    inline Eigen::Vector3f normalAt(const Eigen::Vector2f &uv) const {
      if(tris_.size()<3) Eigen::Vector3f::Zero();
      Eigen::Vector3f v = locate(uv)->normalAt(uv);
      if(v.squaredNorm()>0.00001f) v.normalize();
      ROS_ASSERT(pcl_isfinite(v.sum()));
      return v;
    }

    inline Eigen::Matrix<float,3,2> normal(const Eigen::Vector2f &uv) const {
      if(tris_.size()<3) Eigen::Matrix<float,3,2>::Zero();
      return locate(uv)->normal(uv);
    }

    inline Eigen::Vector3f normalAt2(const Eigen::Vector2f &uv, bool &inside, float &w) const {
      if(tris_.size()<3) Eigen::Vector3f::Zero();
      ParametricSurface::TriSpline2_Fade *tri = locate(uv, inside);
      Eigen::Vector3f bc = tri->UV2BC(uv);
      w = bc.dot(tri->getWeight());
      Eigen::Vector3f v = tri->normalAt2(uv);
      if(v.squaredNorm()>0.00001f) v.normalize();
      ROS_ASSERT(pcl_isfinite(v.sum()));
      return v;
    }

    /*inline Eigen::Matrix4f normalAtUV(const Eigen::Vector2f &uv) const {
      return locate(uv)->normalAtUV(uv);
    }*/

    float operator+=(const Topology &o) {
      ROS_ASSERT(this!=&o);

      float er=0;
      int num=0;

//      ROS_INFO("merge");
      //for(Face_iterator it = del_.faces_begin(); it!=del_.faces_end(); it++)
      //std::cout<<"addr2 "<<&*it->vertex(0)<<"\n";
      //      for(std::map< Vertex*, boost::shared_ptr<POINT> >::iterator it = map_pts_.begin(); it!=map_pts_.end(); it++)
      //        std::cout<<"addr3 "<<it->first<<"\n";
      //      std::cout<<"vertex addr3 "<<map_pts_.begin()->first<<" "<<(--map_pts_.end())->first<<"\n";
      //      ROS_INFO("size %d", map_pts_.size());

      if(pts_.size()<3)
      {
//        ROS_INFO("copy %d", pts_.size());

        for(size_t i=0; i<o.pts_.size(); i++) {
          insertPointWithoutUpdate(*o.pts_[i]);
        }
      }
      else {
//        ROS_INFO("add %d",o.pts_.size());

        std::vector<POINT> temp;
        for(size_t i=0; i<o.pts_.size(); i++) {
          POINT p = *o.pts_[i];

#if 0
          //check for near points
          bool found = false;
          for(size_t j=0; j<pts_.size(); j++) {
            if( (pts_[j]->pt-p.pt).squaredNorm()<0.0001f ) {
#error
              pts_[j]->n2 = (p.weight_*p.n2 + pts_[j]->weight*pts_[j].n2)/(p.weight_+pts_[j].weight);
              const float d = std::acos( pts_[j]->n.dot(p.n) );
              pts_[j]->n = (p.weight_*p.n + pts_[j]->weight*pts_[j].n)/(p.weight_+pts_[j].weight);
              pts_[j]->n.normalize();

              er += (p.pt-pts_[j]->pt).squaredNorm()+0.1f*d*t.squaredNorm();

              pts_[j]->pt = (p.weight_*p.pt + w*pts_[j].pt)/(p.weight_+pts_[j].weight);
              pts_[j]->weight_ += p.weight;

              ++num;
            }
          }

          if(found) continue;
#endif

          bool inside;
          ParametricSurface::TriSpline2_Fade *tri = NULL;
          p.uv = nextPoint(p.pt, &tri);
          //tri = locate(p.uv);

          float w;// = tri->getWeight();
          //inside=true;
          Eigen::Vector3f t = normalAt2(p.uv, inside, w);
          if(inside) {
//            std::cout<<"normal2 "<<t<<"\n";
            p.n2 = (p.weight_*p.n2 + w*t)/(p.weight_+w);
            p.n2.normalize();

//            std::cout<<"w "<<p.weight_<<"\n";
//            std::cout<<"n bef\n"<<p.n<<"\n";

            t = normalAt(p.uv);
            const float d = std::acos( t.dot(p.n) );
            p.n = (p.weight_*p.n + w*t)/(p.weight_+w);
            p.n.normalize();

//            std::cout<<"n after\n"<<p.n<<"\n";

            t = project2world(p.uv);
            p.pt = (p.weight_*p.pt + w*t)/(p.weight_+w);
            p.weight_ += w;

            er += (p.pt-t).squaredNorm()+0.1f*d*t.squaredNorm();
            ++num;
          }
          //ROS_ASSERT( tri==locate(p.uv) );
          tri=locate(p.uv);
//          bool b=tri->inside(p.uv);
          Eigen::Vector2f uv_tmp;
          if(!tri->polating(p.pt, p.uv, inside, p.uv, uv_tmp)) {
            ROS_ASSERT( !locate(p.uv)->complete_inside(p.uv)||!locate(uv_tmp)->complete_inside(uv_tmp) );
            if(locate(p.uv)->inside(p.uv))
              p.uv = uv_tmp;
            ROS_ASSERT( inside==locate(p.uv)->complete_inside(p.uv) );
            if(locate(p.uv)->inside(p.uv))
              continue;
          }
//          std::cout<<b<<"  "<<locate(p.uv)->inside(p.uv)<<std::endl;
//          ROS_ASSERT( pcl_isfinite( p.uv.sum() ) );
          temp.push_back(p);
        }
        for(size_t i=0; i<pts_.size(); i++) {
          Eigen::Vector2f uv = o.nextPoint(pts_[i]->pt);
          bool inside;
          float w;
          Eigen::Vector3f t = o.normalAt2(uv,inside,w);
          if(inside) {
            pts_[i]->n2 = (pts_[i]->weight_*pts_[i]->n2 + w*t)/(pts_[i]->weight_+w);
            pts_[i]->n2.normalize();

            t = o.normalAt(uv);
            const float d = std::acos( t.dot(pts_[i]->n) );
            pts_[i]->n = (pts_[i]->weight_*pts_[i]->n + w*t)/(pts_[i]->weight_+w);
            pts_[i]->n.normalize();

            t = o.project2world(uv);
            pts_[i]->pt = (pts_[i]->weight_*pts_[i]->pt + w*t)/(pts_[i]->weight_+w);
            pts_[i]->weight_ += w;

            er += (pts_[i]->pt-t).squaredNorm()+0.1f*d*t.squaredNorm();
            ++num;
          }
        }
        try {
        for(size_t i=0; i<temp.size(); i++) {
          insertPointWithoutUpdate(temp[i]);
        }
        } catch(...) {
          ROS_ERROR("CGAL error   CGAL error   CGAL error   CGAL error   CGAL error   CGAL error   ");
          return 1000.f;
        }

        if(!num || !pcl_isfinite(er))
          er = 1000.f;
      }
      //      ROS_INFO("size %d", map_pts_.size());
      //      for(std::map< Vertex*, boost::shared_ptr<POINT> >::iterator it = map_pts_.begin(); it!=map_pts_.end(); it++)
      //        std::cout<<"addr3 "<<it->first<<"\n";
      //      std::cout<<"vertex addr3 "<<map_pts_.begin()->first<<" "<<(--map_pts_.end())->first<<"\n";
      update();

      finish();

      return er/std::max(num,1);
    }

    void transform(const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) {
      for(size_t i=0; i<pts_.size(); i++) {
        pts_[i]->transform(rot, tr);
      }

      for(std::vector< boost::shared_ptr<ParametricSurface::TriSpline2_Fade> >::iterator it = tris_.begin();
          it!=tris_.end(); ++it)
      {
        (*it)->transform(rot, tr);
      }
    }

    void print() const {
      /*for(size_t i=0; i<pts_.size(); i++) {
        std::cout<<"uv\n"<<pts_[i]->uv<<"\n";
        std::cout<<"pt\n"<<pts_[i]->pt<<"\n";
      }*/
    }

    void add(cob_3d_marker::MarkerList_Line &ml) const {
      for(std::vector< boost::shared_ptr<ParametricSurface::TriSpline2_Fade> >::const_iterator it = tris_.begin();
          it!=tris_.end(); ++it)
      {
        ParametricSurface::_Line l[6];
        (*it)->test_setup(l);
        //        for(int i=0; i<6; i++)
        //          ml.addLine( l[i].o, l[i].o+l[i].u*0.1f, 0,1,0);
        for(int i=0; i<3; i++) {
          ml.addLine( (*it)->getEdge(i), (*it)->getEdge((i+1)%3) );

          ml.addLine( (*it)->getEdge(i), (*it)->getFade(i)(1,0).head(3) , 0.8f,0.1f,0.1f);
          ml.addLine( (*it)->getFade(i)(1,0).head(3), (*it)->getFade(i)(0,1).head(3) , 0.8f,0.1f,0.1f);
          ml.addLine( (*it)->getFade(i)(0,1).head(3), (*it)->getEdge((i+1)%3) , 0.8f,0.1f,0.1f);
        }
      }
    }

    void add(cob_3d_marker::MarkerList_Triangles &mt, ParametricSurface::TriSpline2_Fade &tri,
             const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c, const int depth=0) const {
      if(depth>=2) {
        Eigen::Matrix<float,3,2> M = tri.normalBC( ((a+b+c)/3).head<2>() );
        M.col(0) = M.col(0).cross(M.col(1));
        M.col(0).normalize();
        M.col(1).fill(0);M.col(1)(2)=1;
        float f=std::abs(M.col(0).dot(M.col(1)));
        mt.addTriangle( tri(a), tri(b), tri(c), f,f,f );
      }
      else {
        add(mt, tri, a, (a+b+c)/3, (a+b)/2, depth+1);
        add(mt, tri, a, (a+b+c)/3, (a+c)/2, depth+1);

        add(mt, tri, b, (a+b+c)/3, (a+b)/2, depth+1);
        add(mt, tri, b, (a+b+c)/3, (b+c)/2, depth+1);

        add(mt, tri, c, (a+b+c)/3, (c+b)/2, depth+1);
        add(mt, tri, c, (a+b+c)/3, (a+c)/2, depth+1);
      }
    }

    void add(cob_3d_marker::MarkerList_Triangles &mt) const {
      for(std::vector< boost::shared_ptr<ParametricSurface::TriSpline2_Fade> >::const_iterator it = tris_.begin();
          it!=tris_.end(); ++it)
      {
        Eigen::Vector3f a,b,c;
        a=b=c=Eigen::Vector3f::Zero();
        a(0)=b(1)=c(2)=1;
        add(mt, *(*it), a,b,c);
      }
    }

    void add(cob_3d_marker::MarkerList_Arrow &ma) const {
      for(std::vector< boost::shared_ptr<ParametricSurface::TriSpline2_Fade> >::const_iterator it = tris_.begin();
          it!=tris_.end(); ++it)
      {
        for(int i=0; i<3; i++) {
          ma.addArrow((*it)->getEdge(i), (*it)->getEdge(i)+(*it)->getNormal(i)*0.25f);

          ma.addArrow((*it)->getEdge(i), (*it)->getEdge(i)+(*it)->getNormal2(i)*0.2f, 1,0,0);
        }
      }
    }

    void add(cob_3d_marker::MarkerList_Text &mt) const {
      for(std::vector< boost::shared_ptr<ParametricSurface::TriSpline2_Fade> >::const_iterator it = tris_.begin();
          it!=tris_.end(); ++it)
      {
        for(int i=0; i<3; i++) {
          char buf[128];
          sprintf(buf,"uv: %f %f", (*it)->getUV(i)(0), (*it)->getUV(i)(1));
          mt.addText((*it)->getEdge(i), buf, 0.02f);
        }
      }
    }

  };

}

#endif /* TOPOLGY_H_ */
