#pragma once

#include <eigen3/Eigen/Dense>

//! collecting descirptive transformations for 6-DOFs
namespace DOF6
{

  template<typename INPUT>
  class TFLink
  {
  public:
    typedef typename INPUT::Scalar TYPE;
    typedef Eigen::Matrix<TYPE,3,3> Matrix;
    typedef Eigen::Matrix<TYPE,4,4> Matrix4;
    typedef Eigen::Matrix<TYPE,3,1> Vector;

  private:

    Matrix covariance_, variance_x_, variance_y_;
    Matrix translation_;
    Vector var_;

    Matrix rot_;
    Vector tr_;

    Vector var_x_, var_y_;
    INPUT sum_x_, sum_y_;
    TYPE rot_sum_, rot_var_, tr_var_, accumlated_weight_, accumlated_weight_t_;
#ifdef DEBUG_
    bool initialized_;
#endif

  public:
    typedef boost::shared_ptr<TFLink<INPUT> > Ptr;

    /**
     * TFLinkObj can represent a point or a plane
     * a object can be reduced to one of these
     * needed is a vector from viewpoint to the next point of object
     */
    struct TFLinkObj
    {
      Matrix translation_M_;
      Vector next_point_;
      float length_;
      Vector rotation_n_;
      float weight_R_,weight_t_;
      bool plane_;

      /*
       * normal is vector from view point to mid of cube or next point on plane
       */
      TFLinkObj(const INPUT &normal,
                const bool plane,
                const bool norm=false, const TYPE wR=1., const TYPE wT=1.)
      :weight_R_(wR),weight_t_(wT), plane_(plane)
      {
        rotation_n_ = normal;

        if(plane_) //plane or point
        {
          length_ = rotation_n_.norm();
          if(norm)
          {
            next_point_.fill(0);
            translation_M_ = translation_M_.Zero(); /// \f$C=QVQ^{-1}\f$
          }
          else
          {
            translation_M_ = normal*normal.transpose()/normal.squaredNorm(); /// \f$C=QVQ^{-1}\f$
            next_point_ = normal;
          }
        }
        else {
          length_ = 1.f;
          next_point_ = normal;
          translation_M_ = translation_M_.Identity();
        }

      }
    };

    TFLink()
    {
      reset();
    }

    void deepCopy(const TFLink &o)
    {
      *this = o;
    }

    void reset()
    {
#ifdef DEBUG_
      initialized_ = false;
#endif
      rot_var_ = 10000; //depends on max. speed
      tr_var_ = 10000;  //depends on max. speed

      rot_ = Matrix::Identity();
      tr_.fill(0);
      accumlated_weight_t_ = accumlated_weight_ = 0;

      rot_sum_ = 0;
      covariance_.fill(0);
      sum_x_.fill(0);
      sum_y_.fill(0);

      translation_.fill(0);
      var_x_.fill(0);
      var_y_.fill(0);

      variance_x_.fill(0);
      variance_y_.fill(0);
    }

    void debug_print() {
      std::cout<<"TRANSLATION MATRIX\n"<<translation_<<"\n";
    }

    void operator()(const TFLinkObj &obj, const TFLinkObj &cor_obj);    /// adding objects
    TFLink operator+(const TFLink &o) const;    /// create chain of tf-links
    void operator+=(const TFLink &o);    /// add tf-links

    TFLink<INPUT> transpose();   /// returns inverse
    void finish();      /// calculate normalized covariance for rotation

    inline Matrix getRotation() const {return rot_;}
    inline Vector getTranslation() const {return tr_;}
    Matrix4 getTransformation() const;  /// returns rotaton+translation

    inline void setRotation(const Matrix &m)    {rot_=m;}
    inline void setTranslation(const Vector &m) {tr_=m;}

    TYPE getRotationVariance() const {return rot_var_;}
    TYPE getTranslationVariance() const {return tr_var_;}

    void check() const;

    inline Ptr makeShared () { return Ptr (new TFLink<INPUT> (*this)); }

    void setTime(const double time_in_sec)
    {
      //not used
    }

    inline bool isRealSource() const {return true;}

    void setVariance(const TYPE Tvar, const Vector &tr, const TYPE Rvar, const Matrix &rot)
    {
      //not used
    }

    TYPE getRotationVariance2() const
    {
      return getRotationVariance();
    }

    TYPE getTranslationVariance2() const
    {
      return getTranslationVariance();
    }

  };

  template<typename INPUT>
  std::ostream &operator<<(std::ostream &os, const TFLink<INPUT> &s)
  {

    os<<"TFLink\n\n";

    os<<"rot\n"<<s.getRotation()<<"\n";
    os<<"tr\n"<<s.getTranslation()<<"\n";
    os<<"var rot: "<<s.getRotationVariance()<<"\n";
    os<<"var tr:  "<<s.getTranslationVariance()<<"\n";

    return os;
  }

  typedef TFLink<Eigen::Vector3f> TFLinkvf;
  typedef TFLink<Eigen::Vector3d> TFLinkvd;

#include "impl/tflink.hpp"
}
