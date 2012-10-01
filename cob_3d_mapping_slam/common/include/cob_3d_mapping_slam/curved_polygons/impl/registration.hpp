
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
void OBJCTXT<_DOF6>::findCorrespondences1(const OBJCTXT &ctxt, std::list<SCOR> &cors,
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
                  obj.getData().canMerge(ctxt.objs_[j]->getData(),0.1f))) )
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
_DOF6 OBJCTXT<_DOF6>::optimizeLink1(const DOF6 &_tf, std::list<SCOR> &cors, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const int depth) const
{
  const float noise = 0.005f;
  DOF6 tf;
  tf.deepCopy(_tf); //copy

  std::cout<<"rot\n"<<::DOF6::EulerAnglesf(rot)<<"\n";
  std::cout<<"tr\n"<<tr<<"\n";
  std::cout<<"thr_rot "<<thr_rot<<"\n";
  std::cout<<"thr_tr  "<<thr_tr<<"\n";

  if(!(thr_rot<1 && thr_tr<1))
    ROS_ERROR("(thr_rot<1 && thr_tr<1) %f %f",thr_rot,thr_tr);

  if(thr_tr<0.01f || thr_rot<0.005f)
  {
    std::cout<<"break because thr_\n";
    return _tf;
  }

  ROS_ASSERT(thr_tr>=0.01f);
  ROS_ASSERT(thr_rot>=0.005f);

  typename DOF6::SOURCE1 &magic_box(*tf.getSource1());
  typedef typename DOF6::SOURCE1::TFLinkObj TFLinkObj;

  magic_box.reset();

  std::map<typename OBJECT::Ptr, unsigned int> cors_a, cors_b;

  int used=0;
  int used2=0;
  for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
  {
    typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
    //typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());

    it->used_ = list.size()>0;

    if(it->used_)
    {
      if(cors_a.find(it->a)==cors_a.end())
        cors_a[it->a]=0;
      if(cors_b.find(it->b)==cors_b.end())
        cors_b[it->b]=0;
      cors_a[it->a]++;
      cors_b[it->b]++;
      ++used;
    }

    if(list.size()) ++used2;
  }
  ROS_INFO("USED %d %d",used,used2);

  for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
  {
    typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
    //typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());

    it->used_ = list.size()>0;

    const float w = std::max(1.f/(cors_a[it->a]*cors_a[it->a]), 1.f/(cors_b[it->b]*cors_b[it->b]));

    for(typename OBJECT::TFLIST::iterator k = list.begin(); k!=list.end(); k++)
    {
      k->a.weight_R_ *= w;
      k->a.weight_t_ *= w;
      k->b.weight_R_ *= w;
      k->b.weight_t_ *= w;
      magic_box(k->a, k->b);
    }
  }

  magic_box.finish();

  if(magic_box.getTranslationVariance()>100)
    magic_box.setTranslation(tr);
  if(magic_box.getRotationVariance()>100)
    magic_box.setRotation(rot);

  //  if(!pcl_isfinite(thr))
  //    //return optimizeLink(_tf, cors, tf.getRotationVariance()+tf.getTranslationVariance(), tf.getRotation(), tf.getTranslation());
  //    return optimizeLink(_tf, cors, 1.5, tf.getRotation(), tf.getTranslation());
  //  else
  if(depth<1 || thr_tr>0.05f || thr_rot>0.05f) {

#ifdef DEBUG_
    //DEBUG
    for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
    {
      if(!it->used_)
      {
        typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
        for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
        {
          Debug::Interface::get().addArrow(k->a.next_point_, k->b.next_point_, 0,255,0);
        }
        //Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->b->getNearestPoint(), 0,255,0);
      }
    }
    //DEBUG
#endif

    if(depth<1 || (thr_tr>0.05f && thr_rot>0.05f))
      return optimizeLink1(_tf, cors, thr_rot*0.6f, thr_tr*0.6f, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
    else if(thr_tr>0.05f)
      return optimizeLink1(_tf, cors, thr_rot, thr_tr*0.6f, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
    else
      return optimizeLink1(_tf, cors, thr_rot*0.6f, thr_tr, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
  }

#ifdef DEBUG_
  //DEBUG
  std::cout<<"WEIGHT LIST\n";
  for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
  {
    if(it->used_)
    {
      float w=0.f;
      typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
      for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
      {
        //        if( k->a.plane_ )
        //        {
        //          Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->a->getNearestPoint()+0.1f*k->a.rotation_n_/k->a.length_,100);
        //          Debug::Interface::get().addArrow(it->b->getNearestPoint(),it->b->getNearestPoint()+0.1f*k->b.rotation_n_/k->b.length_,255,100);
        //        }
        //        else
        //if(k->a.weight_R_>0.05f)
        Debug::Interface::get().addArrow(k->a.next_point_, k->b.next_point_, 128*k->a.weight_R_,128*k->a.weight_R_,128*k->a.weight_R_);
        w += k->a.weight_R_;
        std::cout<<"WEIGHTXX "<<k->a.weight_R_<<"  "<<k->b.weight_R_<<"\n";
      }
      std::cout<<"WEIGHT "<<it->a->getData().getWeight()<<"  "<<it->b->getData().getWeight()<<"  "<<w<<(it->a->getData().isPlane()?" P":" ")<<(it->b->getData().isPlane()?" P":" ")<<"\n";
    }
    else
    {
      Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->b->getNearestPoint(), 0,255,0);
    }
  }
  Debug::Interface::get().addArrow(Eigen::Vector3f::Zero(), tf.getTranslation());

  for(size_t i=0; i<objs_.size(); i++)
  {
    if(!objs_[i]->getData().isPlane()) continue;

    //Debug::Interface::get().addArrow((Eigen::Matrix3f)tf.getRotation()*objs_[i]->getNearestPoint()+tf.getTranslation(),Eigen::Vector3f::Zero(), 100,0,100);
    Debug::Interface::get().addArrow(objs_[i]->getNearestPoint(),Eigen::Vector3f::Zero(), 0,255,0);
  }
  //DEBUG
#endif

  if(//(size_t)used2*4<cors.size() ||
      (_tf.getRotation()-tf.getRotation()).norm()>_tf.getRotationVariance() || (_tf.getTranslation()-tf.getTranslation()).norm()>_tf.getTranslationVariance())
  {
    std::cout<<"OLD\n"<<_tf<<"\n";
    std::cout<<"NEW\n"<<tf<<"\n";
    if((size_t)used2*4<cors.size())
      ROS_INFO("failed reason 1");
    if((_tf.getRotation()-tf.getRotation()).norm()>_tf.getRotationVariance())
      ROS_INFO("failed reason 2");
    if((_tf.getTranslation()-tf.getTranslation()).norm()>_tf.getTranslationVariance())
      ROS_INFO("failed reason 3");
    tf.getSource1()->reset();
  }
  std::cout<<"R1\n"<<_tf.getRotation()<<"\n";
  std::cout<<"R2\n"<< tf.getRotation()<<"\n";
  std::cout<<"rot dist: "<<(_tf.getRotation()-tf.getRotation()).norm()<<"\n";
  std::cout<<"tr dist: "<<(_tf.getTranslation()-tf.getTranslation()).norm()<<"\n";
  std::cout<<"rot dist A: "<<_tf.getRotationVariance()<<"\n";
  std::cout<<"tr dist A: "<<_tf.getTranslationVariance()<<"\n";
  ROS_INFO("-------///\\\\\------");

  return tf;
}

template<typename _DOF6>
_DOF6 OBJCTXT<_DOF6>::optimizeLink2(const DOF6 &_tf, std::list<SCOR> &cors, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const int depth) const
{
  const float noise = 0.005f;
  DOF6 tf;
  tf.deepCopy(_tf); //copy

  std::cout<<"rot\n"<<::DOF6::EulerAnglesf(rot)<<"\n";
  std::cout<<"tr\n"<<tr<<"\n";
  std::cout<<"thr_rot "<<thr_rot<<"\n";
  std::cout<<"thr_tr  "<<thr_tr<<"\n";

  if(!(thr_rot<1 && thr_tr<1))
    ROS_ERROR("(thr_rot<1 && thr_tr<1) %f %f",thr_rot,thr_tr);

  if(thr_tr<0.01f || thr_rot<0.005f)
  {
    std::cout<<"break because thr_\n";
    return _tf;
  }

  ROS_ASSERT(thr_tr>=0.01f);
  ROS_ASSERT(thr_rot>=0.005f);

  typename DOF6::SOURCE1 &magic_box(*tf.getSource1());
  typedef typename DOF6::SOURCE1::TFLinkObj TFLinkObj;

  magic_box.reset();

  std::map<typename OBJECT::Ptr, SCOR> cors_a;

  int used=0;
  int used2=0;
  for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
  {
    typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
    //typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());

    it->used_ = list.size()>0;

    if(it->used_)
    {
      const float s = it->a->getSimilarity( *it->b );
      std::cout<<"similiarity "<<s<<std::endl;
      if(cors_a.find(it->a)==cors_a.end() || s>cors_a[it->a].prob)
      {
        cors_a[it->a].prob=s;
        cors_a[it->a].b=it->b;
        ++used;
      }
    }

    if(list.size()) ++used2;
  }
  ROS_INFO("USED %d %d",used,used2);

  for(typename std::map<typename OBJECT::Ptr, SCOR>::const_iterator it = cors_a.begin(); it!=cors_a.end(); it++)
  {
    typename OBJECT::TFLIST list = it->first->getTFList(*it->second.b, thr_rot+noise, thr_tr+noise, rot, tr);
    //typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());

    //it->used_ = list.size()>0;

    for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
    {
      magic_box(k->a, k->b);
    }
  }

  magic_box.finish();

  if(magic_box.getTranslationVariance()>100)
    magic_box.setTranslation(tr);
  if(magic_box.getRotationVariance()>100)
    magic_box.setRotation(rot);

  if(depth<1 || thr_tr>0.05f || thr_rot>0.05f) {

#ifdef DEBUG_
    //DEBUG
    for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
    {
      if(!it->used_)
      {
        typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
        for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
        {
          Debug::Interface::get().addArrow(k->a.next_point_, k->b.next_point_, 0,255,0);
        }
        //Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->b->getNearestPoint(), 0,255,0);
      }
    }
    //DEBUG
#endif

    if(depth<1 || (thr_tr>0.05f && thr_rot>0.05f))
      return optimizeLink2(_tf, cors, thr_rot*0.6f, thr_tr*0.6f, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
    else if(thr_tr>0.05f)
      return optimizeLink2(_tf, cors, thr_rot, thr_tr*0.6f, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
    else
      return optimizeLink2(_tf, cors, thr_rot*0.6f, thr_tr, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
  }

#ifdef DEBUG_
  //DEBUG
  std::cout<<"WEIGHT LIST\n";
  for(typename std::list<SCOR>::iterator it = cors.begin(); it!=cors.end(); it++)
  {
    if(it->used_)
    {
      float w=0.f;
      typename OBJECT::TFLIST list = it->a->getTFList(*it->b, thr_rot+noise, thr_tr+noise, rot, tr);
      for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
      {
        //if(k->a.weight_R_>0.05f)
        Debug::Interface::get().addArrow(k->a.next_point_, k->b.next_point_, 128*k->a.weight_R_,128*k->a.weight_R_,128*k->a.weight_R_);
        w += k->a.weight_R_;
        std::cout<<"WEIGHTXX "<<k->a.weight_R_<<"  "<<k->b.weight_R_<<"\n";
      }
      std::cout<<"WEIGHT "<<it->a->getData().getWeight()<<"  "<<it->b->getData().getWeight()<<"  "<<w<<(it->a->getData().isPlane()?" P":" ")<<(it->b->getData().isPlane()?" P":" ")<<"\n";
    }
    else
    {
      Debug::Interface::get().addArrow(it->a->getNearestPoint(),it->b->getNearestPoint(), 0,255,0);
    }
  }
  Debug::Interface::get().addArrow(Eigen::Vector3f::Zero(), tf.getTranslation());

  for(size_t i=0; i<objs_.size(); i++)
  {
    if(!objs_[i]->getData().isPlane()) continue;

    //Debug::Interface::get().addArrow((Eigen::Matrix3f)tf.getRotation()*objs_[i]->getNearestPoint()+tf.getTranslation(),Eigen::Vector3f::Zero(), 100,0,100);
    Debug::Interface::get().addArrow(objs_[i]->getNearestPoint(),Eigen::Vector3f::Zero(), 0,255,0);
  }
  //DEBUG
#endif

  if(//(size_t)used2*4<cors.size() ||
      (_tf.getRotation()-tf.getRotation()).norm()>_tf.getRotationVariance() || (_tf.getTranslation()-tf.getTranslation()).norm()>_tf.getTranslationVariance())
  {
    std::cout<<"OLD\n"<<_tf<<"\n";
    std::cout<<"NEW\n"<<tf<<"\n";
    if((size_t)used2*4<cors.size())
      ROS_INFO("failed reason 1");
    if((_tf.getRotation()-tf.getRotation()).norm()>_tf.getRotationVariance())
      ROS_INFO("failed reason 2");
    if((_tf.getTranslation()-tf.getTranslation()).norm()>_tf.getTranslationVariance())
      ROS_INFO("failed reason 3");
    tf.getSource1()->reset();
  }
  std::cout<<"R1\n"<<_tf.getRotation()<<"\n";
  std::cout<<"R2\n"<< tf.getRotation()<<"\n";
  std::cout<<"rot dist: "<<(_tf.getRotation()-tf.getRotation()).norm()<<"\n";
  std::cout<<"tr dist: "<<(_tf.getTranslation()-tf.getTranslation()).norm()<<"\n";
  std::cout<<"rot dist A: "<<_tf.getRotationVariance()<<"\n";
  std::cout<<"tr dist A: "<<_tf.getTranslationVariance()<<"\n";
  ROS_INFO("-------///\\\\\------");

  return tf;
}



template<typename _DOF6>
void OBJCTXT<_DOF6>::findCorrespondences3(const OBJCTXT &ctxt, std::vector<SCOR> &cors,
                                          const DOF6 &tf) {
  map_cors_.clear();

  Eigen::Matrix4f M = tf.getTF4().inverse();
  Eigen::Vector3f t=M.col(3).head<3>();
  Eigen::Matrix3f R=M.topLeftCorner(3,3);

  const float thr = tf.getRotationVariance()+0.05f;

  for(size_t j=0; j<ctxt.objs_.size(); j++)
  {
    OBJECT obj = *ctxt.objs_[j];
    obj.transform(R,t,0,0);

    for(size_t i=0; i<objs_.size(); i++)
      if( (obj.getData().getBB().preassumption(objs_[i]->getData().getBB())>=std::cos(thr+objs_[i]->getData().getBB().ratio())) &&
          obj.intersectsBB(*objs_[i], tf.getRotationVariance(),tf.getTranslationVariance())
          && obj.getData().extensionMatch(objs_[i]->getData(),0.7f,0)
      )
      {
        if(obj.intersectsPts(*objs_[i], tf.getRotationVariance(),tf.getTranslationVariance()) ||
            objs_[i]->intersectsPts(obj, tf.getRotationVariance(),tf.getTranslationVariance()))
        {
          //seccond check
          map_cors_[ctxt.objs_[j]].push_back(cors.size());
          SCOR c;
          c.a = objs_[i];
          c.b = ctxt.objs_[j];
          cors.push_back(c);
        }
      }

  }

#ifdef DEBUG_
  ROS_INFO("found %d correspondences (%d %d)", (int)cors.size(), (int)objs_.size(), (int)ctxt.objs_.size());
#endif

}

template<typename _DOF6>
_DOF6 OBJCTXT<_DOF6>::optimizeLink3(const OBJCTXT &ctxt, std::vector<SCOR> &cors, const DOF6 &_tf, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const int depth) const
{
  const float noiseT = 0.03f, noiseR = 0.01f;
  const float _factor = 0.92f;

  DOF6 tf;
  tf.deepCopy(_tf); //copy

  const bool goon = depth<10 && !(thr_tr<0.01f && thr_rot<0.005f);// || thr_tr>0.05f || thr_rot>0.05f;

  std::cout<<"rot\n"<<::DOF6::EulerAnglesf(rot)<<"\n";
  std::cout<<"tr\n"<<tr<<"\n";
  std::cout<<"thr_rot "<<thr_rot<<"\n";
  std::cout<<"thr_tr  "<<thr_tr<<"\n";

  if(!(thr_rot<1 && thr_tr<1))
    ROS_ERROR("(thr_rot<1 && thr_tr<1) %f %f",thr_rot,thr_tr);

//  ROS_ASSERT(thr_tr>=0.01f);
//  ROS_ASSERT(thr_rot>=0.005f);

  //debug
  int used=0;
  int used2=0;
  int used3=0;

  typename DOF6::SOURCE1 &magic_box(*tf.getSource1());
  typedef typename DOF6::SOURCE1::TFLinkObj TFLinkObj;

  magic_box.reset();

  for(size_t i=0; i<ctxt.objs_.size(); i++)
  {
    typename std::map<typename OBJECT::Ptr,std::vector<size_t> >::const_iterator it = map_cors_.find(ctxt.objs_[i]);
    if(it==map_cors_.end()) continue;

    size_t n = 0;

    typename OBJECT::TFLIST list;
    for(size_t j=0; j<it->second.size(); j++)
    {
      size_t l = list.size();

      cors[it->second[j]].a->addTFList(*ctxt.objs_[i], thr_rot+noiseR, thr_tr+noiseT, rot, tr, list);

      if(list.size()>l) {
        ++used2;
        ++n;
        cors[it->second[j]].used_ = true;
      }
      else
        cors[it->second[j]].used_ = false;

      used3++;
      if(!goon) {
#ifdef DEBUG_
          Debug::Interface::get().addArrow(ctxt.objs_[i]->getNearestPoint(),rot*cors[it->second[j]].a->getNearestPoint()+tr,255,255,255);
#endif
      }
    }

    const float w = std::log(ctxt.objs_[i]->getData().getWeight());
    float wsum = 0.f;
    for(typename OBJECT::TFLIST::const_iterator k = list.begin(); k!=list.end(); k++)
    {
      ++used;
      wsum += k->a.weight_R_ + k->a.weight_t_;
    }

    for(typename OBJECT::TFLIST::iterator k = list.begin(); k!=list.end(); k++)
    {
//      k->a.weight_R_ *= w/wsum;
//      k->a.weight_t_ *= w/wsum;
      //      k->b.weight_R_ *= w/wsum;
      //      k->b.weight_t_ *= w/wsum;
      k->a.weight_R_ /= n*n;
      k->a.weight_t_ /= n*n;
      k->b.weight_R_ /= n*n;
      k->b.weight_t_ /= n*n;

      if(k->a.weight_R_==k->b.weight_R_ && k->a.weight_t_==k->b.weight_t_)
      {
        magic_box(k->a, k->b);

#ifdef DEBUG_
        if(!goon)
        {
          //ROS_INFO("WEIGHT %f %f",k->a.weight_R_,k->a.weight_t_);
          Debug::Interface::get().addArrow(rot*k->a.next_point_+tr,k->b.next_point_,0,0,255);
        }
        else if(goon)
        {
          Debug::Interface::get().addArrow(rot*k->a.next_point_+tr,k->b.next_point_,0,101,0);
        }
#endif

      }
    }
  }

  ROS_INFO("USED %d %d %d",used,used2,used3);

  magic_box.finish();

  std::cout<<"magic_box\n"<<magic_box<<"\n";

  if(magic_box.getTranslationVariance()>100)
    magic_box.setTranslation(tr);
  if(magic_box.getRotationVariance()>100)
    magic_box.setRotation(rot);

  if(goon) {

    if(depth<1 || (thr_tr>0.05f && thr_rot>0.05f))
      return optimizeLink3(ctxt, cors, _tf, thr_rot*_factor, thr_tr*_factor, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
    else if(thr_tr>0.05f)
      return optimizeLink3(ctxt, cors, _tf, thr_rot, thr_tr*_factor, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
    else
      return optimizeLink3(ctxt, cors, _tf, thr_rot*_factor, thr_tr, magic_box.getRotation(), magic_box.getTranslation(), depth+1);
  }

#ifdef DEBUG_
  //DEBUG
  Debug::Interface::get().addArrow(Eigen::Vector3f::Zero(), tf.getTranslation());

  //  for(size_t i=0; i<cors.size(); i++)
  //  {
  //    if(cors[i].used_)
  //      Debug::Interface::get().addArrow(cors[i].a->getNearestPoint(),cors[i].b->getNearestPoint());
  //    else
  //      Debug::Interface::get().addArrow(cors[i].a->getNearestPoint(),cors[i].b->getNearestPoint(), 0,255,0);
  //  }
  //DEBUG
#endif

  if(//(size_t)used2*4<cors.size() ||
      (_tf.getRotation()-tf.getRotation()).norm()>_tf.getRotationVariance() || (_tf.getTranslation()-tf.getTranslation()).norm()>_tf.getTranslationVariance())
  {

    std::cout<<"R1\n"<<_tf.getRotation()<<"\n";
    std::cout<<"R2\n"<< tf.getRotation()<<"\n";
    std::cout<<"rot dist: "<<(_tf.getRotation()-tf.getRotation()).norm()<<"\n";
    std::cout<<"tr dist: "<<(_tf.getTranslation()-tf.getTranslation()).norm()<<"\n";
    std::cout<<"rot dist A: "<<_tf.getRotationVariance()<<"\n";
    std::cout<<"tr dist A: "<<_tf.getTranslationVariance()<<"\n";

    std::cout<<"OLD\n"<<_tf<<"\n";
    std::cout<<"NEW\n"<<tf<<"\n";
    if((size_t)used2*4<cors.size())
      ROS_INFO("failed reason 1");
    if((_tf.getRotation()-tf.getRotation()).norm()>_tf.getRotationVariance())
      ROS_INFO("failed reason 2");
    if((_tf.getTranslation()-tf.getTranslation()).norm()>_tf.getTranslationVariance())
      ROS_INFO("failed reason 3");
    tf.getSource1()->reset();

    return tf;
  }

  return tf;
}
