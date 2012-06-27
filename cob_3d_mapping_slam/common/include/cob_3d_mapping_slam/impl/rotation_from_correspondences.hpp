/*
 * rotation_from_correspondences.hpp
 *
 *  Created on: 28.04.2012
 *      Author: josh
 */

#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Dense>

namespace pcl {

  RotationFromCorrespondences::RotationFromCorrespondences ()
  {
    reset();
  }

  RotationFromCorrespondences::~RotationFromCorrespondences ()
  {
  }

  inline void
  RotationFromCorrespondences::reset ()
  {
    no_of_samples_ = 0;
    accumulated_weight_ = 0.0;
    accumulated_weight2_=(0);
    covariance_.fill(0);
    var_.fill(0);

    va_.fill(0);
    vb_.fill(0);
    vA_.fill(0);
    vB_.fill(0);
  }

  Eigen::Vector3f elmwiseMul(const Eigen::Vector3f& n1, const Eigen::Vector3f& n2)
  {
    Eigen::Vector3f r;
    r(0)=n1(0)*std::abs(n2(0));
    r(1)=n1(1)*std::abs(n2(1));
    r(2)=n1(2)*std::abs(n2(2));
    return r;
  }

  template<typename M>
  M elmwiseDiv(const M& n1, const M& n2)
  {
    M r;
    for(int x=0; x<n1.rows(); x++)
      for(int y=0; y<n1.cols(); y++)
        if(std::abs(n2(x,y))>0.0000001f) r(x,y)=n1(x,y)/n2(x,y);
    return r;
  }

  Eigen::Vector3f elmwiseAbs(const Eigen::Vector3f& n1)
  {
    Eigen::Vector3f r;
    r(0)=std::abs(n1(0));
    r(1)=std::abs(n1(1));
    r(2)=std::abs(n1(2));
    return r;
  }

  inline void
  RotationFromCorrespondences::add (const Eigen::Vector3f& point, const Eigen::Vector3f& corresponding_point,
                                    const Eigen::Vector3f& n1, const Eigen::Vector3f& n2,
                                    const Eigen::Vector3f& cn1, const Eigen::Vector3f& cn2,
                                    float weight, float weight2)
  {
    if (weight==0.0f)
      return;

    ++no_of_samples_;
    accumulated_weight_ += weight;
    //float alpha = weight/accumulated_weight_;

    covariance_ +=  weight*(corresponding_point * point.transpose());
    var_ += weight*(corresponding_point * corresponding_point.transpose());
    static int i=0;


    accumulated_weight2_ += weight2;
    //va_ += weight*(elmwiseMul(corresponding_point,n1)+elmwiseMul(corresponding_point,n2));
    //vb_ += weight*(elmwiseMul(point,n1)+elmwiseMul(point,n2));

    va_ += weight*corresponding_point;
    vb_ += weight*point;
/*
    //eigen composition
    Eigen::Matrix3f Q,V,M;
    Q.col(0)=corresponding_point;
    Q.col(1)=cn1;
    Q.col(2)=cn2;
    V.fill(0);
    V(0,0)=weight2;
    V(1,1)=cn1.norm(); //TODO: expect normalized vector + length?
    V(2,2)=cn2.norm();
    Q.col(0).normalize();
    Q.col(1).normalize();
    Q.col(2).normalize();
    M=V.inverse();//Q*V*Q.transpose();
    vA_ += weight*(M*(corresponding_point));
    std::cout<<"n0:\n"<<(corresponding_point)<<"\n";
    std::cout<<"n1:\n"<<(cn1)<<"\n";
    std::cout<<"n2:\n"<<(cn2)<<"\n";
    std::cout<<"delta:\n"<<(corresponding_point)<<"\nM:\n"<<M<<"\n";
    std::cout<<"res:\n"<<(M*(corresponding_point))<<"\n";

    Q.col(0)=point;
    Q.col(1)=n1;
    Q.col(2)=n2;
    V(1,1)=n1.norm();
    V(2,2)=n2.norm();
    Q.col(0).normalize();
    Q.col(1).normalize();
    Q.col(2).normalize();
    M=V.inverse();//Q*V*Q.transpose();
    vB_ += weight*(M*(point));
*/
    vA_ += weight2*corresponding_point;
    vB_ += weight2*point;
    ++i;
  }

  inline Eigen::Matrix3f
  RotationFromCorrespondences::getTransformation ()
  {
    //  if(no_of_samples_<2)
    //    return Eigen::Matrix3f::Identity();

    std::cout<<"accumulated_weight_\n"<<accumulated_weight_<<"\n";
    std::cout<<"C:\n"<<covariance_<<"\n";
    std::cout<<"vA:\n"<<(vA_*vB_.transpose())<<"\n";
    std::cout<<"vB:\n"<<(va_*vB_.transpose())<<"\n";
    std::cout<<"A:\n"<<(vA_*vB_.transpose())*accumulated_weight_/(accumulated_weight2_*accumulated_weight2_)-((va_*vB_.transpose())+(vA_*vb_.transpose()))/accumulated_weight2_<<"\n";
    std::cout<<"ab:\n"<<(va_*vb_.transpose())/(accumulated_weight_)<<"\n";
    //covariance_ -=  elmwiseDiv<Eigen::Vector3f>(va_,accumulated_weight2_)*elmwiseDiv<Eigen::Vector3f>(vb_,accumulated_weight2_).transpose();//elmwiseDiv<Eigen::Matrix3f>((va_*vb_.transpose()),accumulated_weight2_*accumulated_weight2_.transpose());
    //covariance_ +=  (vA_*vB_.transpose())*accumulated_weight_/(accumulated_weight2_*accumulated_weight2_)-((va_*vB_.transpose())+(vA_*vb_.transpose()))/accumulated_weight2_;
    covariance_ -=  (vA_*vB_.transpose())/(accumulated_weight2_);
    //    var_ -=  elmwiseDiv<Eigen::Matrix3f>((va_*va_.transpose()),accumulated_weight2_*accumulated_weight2_.transpose());

    Eigen::JacobiSVD<Eigen::Matrix<float, 3, 3> > svd (covariance_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix<float, 3, 3>& u = svd.matrixU(),
        & v = svd.matrixV();

    std::cout<<"s:\n"<<svd.singularValues()<<"\n";
    std::cout<<"C:\n"<<covariance_<<"\n";

    Eigen::Matrix<float, 3, 3> s;
    s.setIdentity();
    if (u.determinant()*v.determinant() < 0.0f)
      s(2,2) = -1.0f;

    Eigen::Matrix<float, 3, 3> r = u * s * v.transpose();

    return r;
  }

}  // END namespace
