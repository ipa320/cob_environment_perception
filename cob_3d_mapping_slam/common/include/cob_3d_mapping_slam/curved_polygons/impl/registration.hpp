
#include <eigen3/Eigen/Dense>

template<typename _DOF6>
typename _DOF6::TYPE OBJCTXT<_DOF6>::check_assumption(typename SCOR_MEMBER::Ptr m1, typename SCOR_MEMBER::Ptr m2) const
{
  typename DOF6::TYPE w_sum=(typename DOF6::TYPE)0, p=(typename DOF6::TYPE)0;
  const typename DOF6::TYPE metric=(typename DOF6::TYPE)0.002;

  size_t goods=0;
  for(typename SCOR_MEMBER::MAP::const_iterator it = m1->distances_.begin(); it!=m1->distances_.end(); it++)
  {
    typename DOF6::TYPE mi = (typename DOF6::TYPE)10000;

    for(size_t j=0; j<it->first->candidates_.size(); j++)
    {
      typename SCOR_MEMBER::MAP::const_iterator it2 = m2->distances_.find(it->first->candidates_[j]);

      if(it2!=m2->distances_.end())
      {

        for(size_t k=0; k<it->second.size(); k++)
        {
          for(size_t l=0; l<it2->second.size(); l++)
          {
            //ROS_INFO("d %f %f",it->second[k], it2->second[l]);
            mi = std::min(mi, std::abs(it->second[k] - it2->second[l]) );
          }
        }
      }
    }

//    ROS_INFO("candid with best %f",mi);

    if(mi<10)
    {
      p += std::pow(metric, mi);
      w_sum += 1;
    }

    if(std::pow(metric, mi)>0.7f)
      goods++;
  }
  if(w_sum) p/=w_sum;

#ifdef DEBUG_
  ROS_INFO("Assumption is p=%f (%d, %d) goods=%d",p,(int)m1->candidates_.size(),(int)m2->candidates_.size(),goods);
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

      //if( obj.isReachable(*ctxt.objs_[j],tf.getRotationVariance(),tf.getTranslationVariance()) )
      if(obj.isReachable(*ctxt.objs_[j],tf.getRotationVariance(),tf.getTranslationVariance())
          || (
          (
              obj.getData().intersectsBB(ctxt.objs_[j]->getData(), 0.1f /*metres*/) &&
              obj.getData().canMerge(ctxt.objs_[j]->getData()))) )
      {
        members.back()->candidates_.push_back(members2[j]);
        members2[j]->candidates_.push_back(members.back());
      }
    }
  }

  for(size_t i=0; i<ctxt.objs_.size(); i++) {
    //if(members2[i]->candidates_.size()<1) continue;
    for(size_t j=i+1; j<ctxt.objs_.size(); j++) {
      //if(members2[j]->candidates_.size()<1) continue;

      std::vector<typename DOF6::TYPE> d = ctxt.objs_[i]->getDistance(*ctxt.objs_[j]);

      members2[i]->distances_[members2[j]] = d;
      members2[j]->distances_[members2[i]] = d;
    }
  }

  for(size_t i=0; i<objs_.size(); i++) {
    //if(members[i]->candidates_.size()<1) continue;
    for(size_t j=i+1; j<objs_.size(); j++) {
      //if(members[j]->candidates_.size()<1) continue;

      std::vector<typename DOF6::TYPE> d = objs_[i]->getDistance(*objs_[j]);

      members[i]->distances_[members[j]] = d;
      members[j]->distances_[members[i]] = d;
    }
  }

  for(size_t i=0; i<members.size(); i++)
  {
    for(size_t j=0; j<members[i]->candidates_.size(); j++)
      if(check_assumption(members[i],members[i]->candidates_[j])>0.49f)
      {
        SCOR c;
        c.a = members[i]->obj_;
        c.b = members[i]->candidates_[j]->obj_;
        cors.push_back(c);
      }
#ifdef DEBUG_
    ROS_INFO("---------------------");
#endif
  }

#ifdef DEBUG_
  ROS_INFO("found %d correspondences (%d %d)", (int)cors.size(), (int)objs_.size(), (int)ctxt.objs_.size());
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
  const float noise = 0.005f;
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
  for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
  {
    typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
    //typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());

    it->used_ = list.size()>0;

    for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
    {
      std::cout<<"A\n"<<k->a.translation_M_<<"\n";
      std::cout<<"B\n"<<k->b.translation_M_<<"\n";
      magic_box(k->a, k->b);
      ++used;
    }

    if(list.size()) ++used2;

    ROS_INFO("----------------------");
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
  if(depth<1 || thr_rot>0.06f)
    return optimizeLink(_tf, cors, thr_rot*0.6f, thr_tr*0.6f, tf.getRotation(), tf.getTranslation(), depth+1);

#ifdef DEBUG_
  //DEBUG
  for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
  {
    if(it->used_)
    {
      typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
      for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
      {
//        if( k->a.plane_ )
//        {
//          Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->a->getNearestPoint()+0.1f*k->a.rotation_n_/k->a.length_,100);
//          Debug::Interface::get().addArrow(it->b->getNearestPoint(),it->b->getNearestPoint()+0.1f*k->b.rotation_n_/k->b.length_,255,100);
//        }
//        else
          Debug::Interface::get().addArrow(k->a.next_point_, k->b.next_point_);
      }
    }
    else
    {
      Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->b->getNearestPoint(), 100,255,0);
    }
  }
  Debug::Interface::get().addArrow(Eigen::Vector3f::Zero(), tf.getTranslation());

  for(size_t i=0; i<objs_.size(); i++)
  {
    if(!objs_[i]->getData().isPlane()) continue;

    Debug::Interface::get().addArrow((Eigen::Matrix3f)tf.getRotation()*objs_[i]->getNearestPoint()+tf.getTranslation(),Eigen::Vector3f::Zero(), 100,0,100);
    //Debug::Interface::get().addArrow(objs_[i]->getNearestPoint(),Eigen::Vector3f::Zero(), 100,0,100);
  }
  //DEBUG
#endif

  if((size_t)used2*4<cors.size())
    tf.getSource1()->reset();

  return tf;
}
