

#include <eigen3/Eigen/Dense>

template<typename _DOF6>
void OBJCTXT<_DOF6>::findCorrespondences(const OBJCTXT &ctxt, std::list<SCOR> &cors,
                                         const DOF6 &tf) const
{
  std::cout<<tf<<"\n";

  for(size_t i=0; i<objs_.size(); i++) {
    if(!objs_[i]) continue;

    OBJECT obj = *objs_[i];
    obj.transform(tf.getRotation(),tf.getTranslation(),0,0);

    SCOR c;
    bool found=false;
    float dmin=100.f;

    for(size_t j=0; j<ctxt.objs_.size(); j++) {
      if(!ctxt.objs_[j]) continue;

      if( (obj)&(*ctxt.objs_[j]) )
      {
        float d=(objs_[i]->getNearestPoint()-ctxt.objs_[j]->getNearestPoint()).squaredNorm();
        //if(d<dmin)
        {
          dmin=d;
          c.a = objs_[i];
          c.b = ctxt.objs_[j];
          found=true;
          cors.push_back(c);

          std::cout<<"a1:\n"<<c.a->getNearestPoint()<<"\n";
          std::cout<<"b1:\n"<<c.b->getNearestPoint()<<"\n";
          std::cout<<"a2:\n"<<c.a->getData().getFeatures()[2].v_<<"\n";
          std::cout<<"b2:\n"<<c.b->getData().getFeatures()[2].v_<<"\n";
        }

#ifdef DEBUG_
        if(i!=j)
          ROS_INFO("false cors. in sim");
#endif
      }
    }

//    if(found)
//      cors.push_back(c);
  }

#ifdef DEBUG_
  ROS_INFO("found %d correspondences", (int)cors.size());
#endif
}

template<typename _DOF6>
_DOF6 OBJCTXT<_DOF6>::optimizeLink(const DOF6 &_tf, std::list<SCOR> &cors, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const int depth) const
{
  DOF6 tf;
  tf.deepCopy(_tf); //copy

  std::cout<<"rot\n"<<::DOF6::EulerAnglesf(rot)<<"\n";
  std::cout<<"tr\n"<<tr<<"\n";
  std::cout<<"thr_rot "<<thr_rot<<"\n";
  std::cout<<"thr_tr  "<<thr_tr<<"\n";

  typename DOF6::SOURCE1 &magic_box(*tf.getSource1());
  typedef typename DOF6::SOURCE1::TFLinkObj TFLinkObj;

  int used=0;
  int used2=0;
  for(typename std::list<SCOR>::const_iterator it = cors.begin(); it!=cors.end(); it++)
  {
    typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+0.025f, thr_tr+0.025f, rot, tr);
    //typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+0.025f, thr_tr+0.025f, Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());

    for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
    {
      std::cout<<"A\n"<<k->a.translation_M_<<"\n";
      std::cout<<"B\n"<<k->b.translation_M_<<"\n";
      magic_box(k->a, k->b);
      ++used;
    }

    if(list.size()) ++used2;
  }
  ROS_INFO("USED %d %d",used,used2);

  magic_box.finish();

  if(magic_box.getTranslationVariance()>100)
    magic_box.setTranslation(tr);
  if(magic_box.getRotationVariance()>100)
    magic_box.setRotation(rot);

//  if(!pcl_isfinite(thr))
//    //return optimizeLink(_tf, cors, tf.getRotationVariance()+tf.getTranslationVariance(), tf.getRotation(), tf.getTranslation());
//    return optimizeLink(_tf, cors, 1.5, tf.getRotation(), tf.getTranslation());
//  else
    if(depth<3 || thr_rot>0.07f)
        return optimizeLink(_tf, cors, thr_rot*0.75f, thr_tr*0.75f, tf.getRotation(), tf.getTranslation(), depth+1);

#ifdef DEBUG_
    //DEBUG
    for(typename std::list<SCOR>::const_iterator it = cors.begin(); it!=cors.end(); it++)
    {
//      typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+0.025f, thr_tr+0.025f, tf.getRotation(), tf.getTranslation());
      typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+0.025f, thr_tr+0.025f, rot, tr);

      if(list.size()>0)
      {
        Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->b->getNearestPoint());
      }
      else
      {
        Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->b->getNearestPoint(), 100,255,0);
      }
    }
    Debug::Interface::get().addArrow(Eigen::Vector3f::Zero(), tf.getTranslation());
    //DEBUG
#endif

  return tf;
}
