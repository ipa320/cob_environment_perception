
#include <eigen3/Eigen/Dense>

template<typename _DOF6>
typename _DOF6::TYPE OBJCTXT<_DOF6>::check_assumption(typename SCOR_MEMBER::Ptr m1, typename SCOR_MEMBER::Ptr m2) const
{
  typename DOF6::TYPE w_sum=(typename DOF6::TYPE)0, p=(typename DOF6::TYPE)0;
  const typename DOF6::TYPE metric=(typename DOF6::TYPE)0.002;

  for(size_t i=0; i<m1->distances_.size(); i++)
  {
    typename DOF6::TYPE mi = (typename DOF6::TYPE)10000;
    for(size_t j=0; j<m2->candidates_.size(); j++)
    {
      mi = std::min(mi, std::abs(m1->distances_[i].d_-m2->candidates_[j].d_) );
      ROS_INFO("candid with %f %f",m1->distances_[i].d_,m2->candidates_[j].d_);
    }

    if(mi<10)
    {
      p += std::pow(metric, mi);
      w_sum += 1;
    }
  }
  if(w_sum) p/=w_sum;

#ifdef DEBUG_
  ROS_INFO("Assumption is p=%f (%d, %d)",p,m1->candidates_.size(),m2->candidates_.size());
#endif

  return p;
}

template<typename _DOF6>
void OBJCTXT<_DOF6>::findCorrespondences(const OBJCTXT &ctxt, std::list<SCOR> &cors,
                                         const DOF6 &tf) const
                                         {
  std::vector<typename SCOR_MEMBER::Ptr> members, members2;

  for(size_t j=0; j<ctxt.objs_.size(); j++) {
    members2.push_back(typename SCOR_MEMBER::Ptr(new SCOR_MEMBER));
    members2.back()->obj_ = ctxt.objs_[j];
  }

  for(size_t i=0; i<objs_.size(); i++) {
    members.push_back(typename SCOR_MEMBER::Ptr(new SCOR_MEMBER));
    if(!objs_[i]) continue;

    members.back()->obj_ = objs_[i];

    OBJECT obj = *objs_[i];
    obj.transform(tf.getRotation(),tf.getTranslation(),0,0);

    for(size_t j=0; j<ctxt.objs_.size(); j++) {
      if(!ctxt.objs_[j]) continue;

      if( obj.isReachable(*ctxt.objs_[j],tf.getRotationVariance(),tf.getTranslationVariance()) )
      {
        SCOR_DISTANCES d;
        d.d_ = ctxt.objs_[i]->getDistance(*ctxt.objs_[j]);

        d.to_ = members2[j];
        members.back()->candidates_.push_back(d);
        d.to_ = members.back();
        members2[j]->candidates_.push_back(d);
      }
    }
  }

  for(size_t i=0; i<ctxt.objs_.size(); i++) {
    //if(members2[i]->candidates_.size()<1) continue;
    for(size_t j=i+1; j<ctxt.objs_.size(); j++) {
      //if(members2[j]->candidates_.size()<1) continue;

      SCOR_DISTANCES d;
      d.d_ = ctxt.objs_[i]->getDistance(*ctxt.objs_[j]);

      d.to_ = members2[j];
      members2[i]->distances_.push_back(d);
      d.to_ = members2[i];
      members2[j]->distances_.push_back(d);
    }
  }

  for(size_t i=0; i<objs_.size(); i++) {
    //if(members[i]->candidates_.size()<1) continue;
    for(size_t j=i+1; j<objs_.size(); j++) {
      //if(members[j]->candidates_.size()<1) continue;

      SCOR_DISTANCES d;
      d.d_ = objs_[i]->getDistance(*objs_[j]);

      d.to_ = members[j];
      members[i]->distances_.push_back(d);
      d.to_ = members[i];
      members[j]->distances_.push_back(d);
    }
  }

  for(size_t i=0; i<members.size(); i++)
  {
    for(size_t j=0; j<members[i]->candidates_.size(); j++)
      if(check_assumption(members[i],members[i]->candidates_[j].to_)>0.7)
      {
        SCOR c;
        c.a = members[i]->obj_;
        c.b = members[i]->candidates_[j].to_->obj_;
        cors.push_back(c);
      }
#ifdef DEBUG_
    ROS_INFO("---------------------");
#endif
  }

#ifdef DEBUG_
  ROS_INFO("found %d correspondences", (int)cors.size());
#endif

  return;

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
