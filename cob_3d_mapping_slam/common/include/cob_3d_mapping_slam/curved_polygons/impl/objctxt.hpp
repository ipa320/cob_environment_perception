

#include <eigen3/Eigen/Dense>

template<typename _DOF6>
bool OBJCTXT<_DOF6>::registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate){

#ifdef DEBUG_
  ROS_INFO("registration %d %d",(int)objs_.size(), (int)ctxt.objs_.size());
#endif

  //1. correspondences
  std::list<SCOR> cors;
  findCorrespondences(ctxt, cors, tf);

  //2. optimize
  tf.getSource1()->reset();
  //tf = optimizeLink(tf, cors);
  tf = optimizeLink(tf, cors, tf.getRotationVariance(), tf.getTranslationVariance(), tf.getRotation(), tf.getTranslation());

  std::cout<<tf<<"\n";

  //set result to time
  // if one registration failed
  // it can be retrieved over time
  tf.setVariance(  tf.getTranslationVariance(), tf.getTranslation(), tf.getRotationVariance(), tf.getRotation() );

  std::cout<<tf<<"\n";

  return true;

  return false;
}


template<typename _DOF6>
bool OBJCTXT<_DOF6>::add(const OBJCTXT &ctxt, const DOF6 &tf)
{
  const size_t old = objs_.size();

  ROS_INFO("add ctxt");
  std::cout<<tf<<"\n";

  for(size_t j=0; j<ctxt.objs_.size(); j++) {
    if(!ctxt.objs_[j]) continue;

    typename OBJECT::Ptr o=ctxt.objs_[j]->makeShared();
    o->transform(Eigen::Matrix3f::Identity(),-tf.getTranslation(),0,0);
    o->transform(((Eigen::Matrix3f)tf.getRotation()).inverse(),Eigen::Vector3f::Zero(),
                 tf.getRotationVariance(), tf.getTranslationVariance());
    //o->transform(((Eigen::Matrix3f)tf.getRotation()).transpose(),-tf.getTranslation());

    bool found=false;
    for(size_t i=0; i<objs_.size(); i++) {
      if(!objs_[i]) continue;

      if( ((*objs_[i])|(*o)) || ((*objs_[i])&(*o)) )
      {
        found = true;

        typename OBJECT::TFLIST list = objs_[i]->getTFList(*ctxt.objs_[j], tf.getRotationVariance()+0.1, tf.getTranslationVariance()+0.1, tf.getRotation(), tf.getTranslation());
        if(list.size()>0)
        {
          ROS_INFO("update object");
          (*objs_[i]) += *o;
        }
        else
          ROS_INFO("ignoring object");
      }
    }

    if(!found)
    {
      ROS_INFO("adding object");

      *this += o;
    }

  }

  return true;
}
