template<typename INPUT>
void TFLink<INPUT>::operator()(const TFLinkObj &obj, const TFLinkObj &cor_obj)
{
  ROS_ASSERT(obj.plane_==cor_obj.plane_);
  ROS_ASSERT(obj.weight_R_==cor_obj.weight_R_);
  ROS_ASSERT(obj.weight_t_==cor_obj.weight_t_);

  if(!obj.length_ || !obj.length_ || !pcl_isfinite( obj.weight_R_ ) || !pcl_isfinite( cor_obj.weight_R_ ) || !pcl_isfinite( obj.weight_t_ ) || !pcl_isfinite( cor_obj.weight_t_ ))
    return;

  ROS_ASSERT(pcl_isfinite( obj.next_point_.sum() ));
  ROS_ASSERT(pcl_isfinite( cor_obj.next_point_.sum() ));
  ROS_ASSERT(pcl_isfinite( obj.rotation_n_.sum() ));
  ROS_ASSERT(pcl_isfinite( cor_obj.rotation_n_.sum() ));

  ROS_ASSERT(pcl_isfinite( obj.weight_R_ ));
  ROS_ASSERT(pcl_isfinite( cor_obj.weight_R_ ));
  ROS_ASSERT(pcl_isfinite( obj.weight_t_ ));
  ROS_ASSERT(pcl_isfinite( cor_obj.weight_t_ ));

  if(!obj.plane_) {
    sum_x_ += obj.weight_R_*cor_obj.next_point_;
    sum_y_ += obj.weight_R_*obj.next_point_;
    rot_sum_ += obj.weight_R_;
  }

  variance_x_ += obj.weight_R_*cor_obj.rotation_n_ * cor_obj.rotation_n_.transpose() /(cor_obj.length_*cor_obj.length_);
  if(!obj.plane_||cor_obj.rotation_n_.dot(obj.rotation_n_)>-0.7*(cor_obj.length_*obj.length_)) {
    covariance_ += obj.weight_R_*cor_obj.rotation_n_ * obj.rotation_n_.transpose() /(cor_obj.length_*obj.length_);
    variance_y_ += obj.weight_R_*obj.rotation_n_ * obj.rotation_n_.transpose() /(obj.length_*obj.length_);
  }
  else {
    covariance_ -= obj.weight_R_*cor_obj.rotation_n_ * obj.rotation_n_.transpose() /(cor_obj.length_*obj.length_);
    variance_y_ -= obj.weight_R_*obj.rotation_n_ * obj.rotation_n_.transpose() /(obj.length_*obj.length_);
  }
  accumlated_weight_ += obj.weight_R_;

  translation_ += obj.weight_t_*cor_obj.translation_M_;
  var_x_ += obj.weight_t_*cor_obj.next_point_;
  var_y_ += obj.weight_t_*obj.next_point_;
  accumlated_weight_t_ += obj.weight_t_;
}

template<typename INPUT>
TFLink<INPUT> TFLink<INPUT>::operator+(const TFLink &o) const
{
  TFLink<INPUT> r;
#if 0
  //std::cout<<"test\n"<<variance_x_.inverse()*covariance_<<"\n";

  std::cout<<"test1\n"<<covariance_<<"\n";
  std::cout<<"test2\n"<<getRotation()*variance_x_<<"\n";
  std::cout<<"test3\n"<<getRotation()*variance_y_<<"\n";

  std::cout<<"test1\n"<<o.covariance_<<"\n";
  std::cout<<"test2\n"<<o.getRotation()*o.variance_x_<<"\n";
  std::cout<<"test3\n"<<o.getRotation()*o.variance_y_<<"\n";
  /*std::cout<<"test\n"<<variance_y_<<"\n";
  std::cout<<"test\n"<<covariance_*getRotation().transpose()<<"\n";
  std::cout<<"test\n"<<getRotation().inverse()*covariance_<<"\n";
  std::cout<<"test\n"<<getRotation()*variance_y_<<"\n";*/

  Matrix t=o.covariance_;
  t.normalize();
  r.covariance_ = variance_y_ * o.getRotation().transpose();// * o.covariance_.inverse().transpose() * covariance_;
  r.covariance_ = covariance_*o.covariance_;
  r.covariance_ = variance_x_*t.inverse().transpose()*t;
  r.covariance_ = o.covariance_*covariance_.transpose();

  r.covariance_ = variance_x_.transpose()*covariance_.inverse().transpose()*o.covariance_;

  //r.covariance_ = o.covariance_*covariance_.transpose();

  r.covariance_ = o.getRotation()*covariance_;

  std::cout<<"test\n"<<(o.getRotation()*getRotation()).inverse()<<"\n";

  Matrix A=o.covariance_;
  Matrix B=covariance_;
  A.normalize();
  B.normalize();
  r.covariance_ = variance_y_*covariance_*variance_y_.inverse()*o.covariance_*o.variance_y_.inverse();

  std::cout<<"V1\n"<<variance_y_<<"\n";
  std::cout<<"U1\n"<<variance_x_<<"\n";
  std::cout<<"V1\n"<<variance_y_.inverse()<<"\n";
  std::cout<<"V2\n"<<o.variance_y_.inverse()<<"\n";
  std::cout<<"COV1\n"<<covariance_<<"\n";
  std::cout<<"COV2\n"<<o.covariance_<<"\n";
  std::cout<<"COV\n"<<r.covariance_<<"\n";

  r.covariance_ = o.getRotation()*covariance_; //works (1,2)
  //r.covariance_ = o.covariance_*o.variance_y_.inverse()*covariance_; //works (1), not(2)

  r.covariance_ = o.covariance_*getRotation(); //works (1,2)

#endif

  r.covariance_ = getRotation()*o.covariance_ + covariance_*o.getRotation();// + o.covariance_*getRotation(); //works (1,2)
  r.accumlated_weight_ = std::min(accumlated_weight_,o.accumlated_weight_);
  r.variance_x_ = variance_x_ + getRotation()*o.variance_x_*getRotation().transpose();
  r.variance_y_ = o.variance_y_ + o.getRotation().transpose()*variance_y_*o.getRotation();

//  r.var_x_ = o.var_x_ + o.getRotation()*var_y_ + o.getTranslation()*accumlated_weight_t_;
//  r.var_y_ =   var_y_ + getRotation().transpose()*o.var_x_ - getTranslation()*o.accumlated_weight_t_;
//  r.translation_ = o.getRotation()*translation_ + o.translation_*getRotation();
//  r.var_x_ = o.getRotation()*var_x_ + o.getTranslation()*accumlated_weight_t_;
//  r.var_y_ = var_y_;
//  r.translation_ = o.getRotation()*translation_*o.getRotation().transpose();
  r.accumlated_weight_t_ = accumlated_weight_t_ + o.accumlated_weight_t_;

//  r.var_x_ = o.translation_*var_x_ + translation_*getRotation()*o.var_x_;
//  r.var_y_ = o.translation_*o.getRotation().transpose()*var_y_ + translation_*o.var_y_;
//  r.translation_ = translation_ * o.translation_;

//  r.var_x_ = o.getRotation()*(var_x_ +  translation_*getRotation()*o.getTranslation());
//  r.var_y_ = var_y_;
//  r.translation_ = o.getRotation()*translation_;

//  r.var_x_ = getRotation()*o.var_x_+o.translation_*getTranslation();
//  r.var_y_ = o.var_y_;
//  r.translation_ = o.translation_;


  //TODO: wrong
//  r.var_x_ = o.getRotation()*(var_x_ +  translation_*getRotation()*o.getTranslation()) + getRotation()*o.var_x_+o.translation_*getTranslation();
//  r.var_y_ = var_y_ + o.var_y_;
//  r.translation_ = o.getRotation()*translation_ + o.translation_;

    r.var_x_ = var_x_ + translation_ * getRotation() * o.getTranslation();
    r.var_y_ = o.getRotation().transpose()*var_y_;
    r.translation_ = translation_;


//  r.var_x_ = getRotation() * o.var_x_;// + getRotation() * o.translation_ * getTranslation();
//  r.var_y_ = o.var_y_;
//  r.translation_ = getRotation() * o.translation_;

//      r.var_x_ = o.var_x_ + o.translation_ * o.getRotation() * getTranslation();
//      r.var_y_ = getRotation().transpose()*o.var_y_;
//      r.translation_ = getRotation().transpose()*o.translation_.transpose()*getRotation();

  //#error
  //  Vector temp = var_x_ - rot_*var_y_;
  //  tr_ = translation_.fullPivLu().solve(temp);

  r.finish();

  //r.tr_ = tr_ + getRotation()*o.tr_;
  r.rot_var_= rot_var_+ o.rot_var_;
  r.tr_var_ = tr_var_ + o.tr_var_;

  return r;
}

template<typename INPUT>
void TFLink<INPUT>::operator+=(const TFLink &o)
{

#ifdef DEBUG_OUTPUT_
  std::cout<<"+=VAR X1\n"<<var_x_<<"\n";
  std::cout<<"+=VAR X2\n"<<o.var_x_<<"\n";
  std::cout<<"+=VAR Y1\n"<<var_y_<<"\n";
  std::cout<<"+=VAR Y2\n"<<o.var_y_<<"\n";
  std::cout<<"+=VAR T1\n"<<translation_<<"\n";
  std::cout<<"+=VAR T2\n"<<o.translation_<<"\n";
#endif

  covariance_ += o.covariance_;
  variance_x_ += o.variance_x_;
  variance_y_ += o.variance_y_;
  accumlated_weight_ += o.accumlated_weight_;
  var_x_ += o.var_x_;
  var_y_ += o.var_y_;
  translation_ += o.translation_;
  accumlated_weight_t_ += o.accumlated_weight_t_;


  finish();
}

template<typename INPUT>
TFLink<INPUT>  TFLink<INPUT>::transpose() const {
  TFLink<INPUT> r;

  r.covariance_ = covariance_.transpose();
  r.variance_x_ = variance_y_;
  r.variance_y_ = variance_x_;

  r.translation_ = rot_.transpose()*translation_.transpose()*rot_;
  r.var_x_ = var_y_;
  r.var_y_ = var_x_;

  r.rot_ = rot_.transpose();
  r.tr_ = -r.rot_*tr_;

  return r;
}

template<typename INPUT>
void TFLink<INPUT>::finish() {

  //  std::cout<<"variance_\n"<<covariance_<<"\n";
  if(rot_sum_) {
    covariance_ -= (sum_x_*sum_y_.transpose())/rot_sum_;
    variance_x_ -= (sum_x_*sum_x_.transpose())/rot_sum_;
    variance_y_ -= (sum_y_*sum_y_.transpose())/rot_sum_;

    sum_x_.fill(0);
    sum_y_.fill(0);
    rot_sum_=0; //TODO: check this line
  }

  // ------------- ROTATION ------------------

  //  std::cout<<"variance_\n"<<covariance_<<"\n";

  Eigen::JacobiSVD<Matrix> svd (covariance_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Matrix& u = svd.matrixU(),
      & v = svd.matrixV();

  Matrix s = Matrix::Identity();
  if (u.determinant()*v.determinant() < 0.0f)
    s(2,2) = -1.0f;

  rot_ = u * s * v.transpose();

  ROS_ASSERT(pcl_isfinite(rot_.sum()));

  rot_var_ = std::max(
      /*svd.singularValues().squaredNorm()<0.01f*accumlated_weight_*accumlated_weight_ || */
      svd.singularValues()(1)*svd.singularValues()(1)<=0.00001f*svd.singularValues().squaredNorm() ? 100000. : 0.,
      //svd.singularValues().squaredNorm()<0.01f ? 100000. : 0.,
          M_PI*sqrtf((variance_y_-(rot_.transpose()*variance_x_*rot_)).squaredNorm()/variance_y_.squaredNorm()));

  if(!pcl_isfinite(rot_var_))
    rot_var_ = 100000;

#ifdef DEBUG_OUTPUT_
  std::cout<<"ROT\n"<<rot_<<"\n";
  std::cout<<"SING VALS\n"<<svd.singularValues()<<"\n";
    std::cout<<"ROT_DIS "<<sqrtf((variance_y_-(rot_.transpose()*variance_x_*rot_)).squaredNorm()/variance_y_.squaredNorm())<<"\n";
    std::cout<<"ROT1 \n"<<(variance_y_)<<"\n";
    std::cout<<"ROT2 \n"<<((rot_.transpose()*variance_x_*rot_))<<"\n";
#endif

  // ------------- TRANSLATION ------------------

  Vector temp = var_x_ - rot_*var_y_;
  tr_ = translation_.fullPivLu().solve(temp);
  tr_var_ = (temp-translation_*tr_).norm() + tr_.norm()*rot_var_/(2*M_PI);

  ROS_ASSERT(pcl_isfinite(temp.sum()));

#ifdef DEBUG_OUTPUT_
  std::cout<<"rot_var. "<<rot_var_<<"\n";
  std::cout<<"TRANSLATION MATRIX\n"<<translation_<<"\n";
  std::cout<<"TRANSLATION VECTOR1\n"<<temp<<"\n";
  std::cout<<"TRANSLATION VECTOR2\n"<<translation_*tr_<<"\n";
  std::cout<<"det. "<<translation_.determinant()<<"\n";
  std::cout<<"should det. "<<(0.2*accumlated_weight_*accumlated_weight_)<<"\n";
#endif

  if(translation_.determinant()<(TYPE)0.04*accumlated_weight_*accumlated_weight_) // rang to low
    tr_var_ += 10000;

  if(!pcl_isfinite(tr_var_) || !pcl_isfinite(tr_.sum()))
    tr_var_ = 100000;
  if(!pcl_isfinite(tr_.sum())
#ifndef ATOM_TESTING_
      ||tr_var_>=1000
#endif
  )
    tr_.fill(0);

}

template<typename INPUT>
void TFLink<INPUT>::check() const {
  Matrix R = getRotation();

#ifdef DEBUG_OUTPUT_
  std::cout<<"V1\n"<<variance_y_<<"\n";
  std::cout<<"U1\n"<<variance_x_<<"\n";

  std::cout<<"V2\n"<<R.transpose()*variance_x_*R<<"\n";
  std::cout<<"U2\n"<<R*variance_y_*R.transpose()<<"\n";

  std::cout<<"C\n"<<(variance_y_-(R.transpose()*variance_x_*R)).norm()<<"\n";
  std::cout<<"C\n"<<(variance_x_-(R*variance_y_*R.transpose())).norm()<<"\n";
#endif

}

template<typename INPUT>
typename TFLink<INPUT>::Matrix4 TFLink<INPUT>::getTransformation() const {
  Matrix rot = getRotation();

  //std::cout<<"delta\n"<<var_x_ - rot*var_y_<<"\n";
  //std::cout<<"translation:\n"<<t<<"\n";

  Matrix4 r=Matrix4::Identity();
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++)
      r(i,j) = rot(i,j);
    r.col(3)(i)= getTranslation()(i);
  }

  return r;
}
