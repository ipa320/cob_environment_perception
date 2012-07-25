

template<typename OBJECT_CONTEXT>
bool Node<OBJECT_CONTEXT>::registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate)
{
  //1. find correspondences
  //2. register
  return ctxt_.registration(ctxt, tf, probability_success_rate, probability_error_rate);
}


template<typename OBJECT_CONTEXT>
bool Node<OBJECT_CONTEXT>::addCtxt(const OBJCTXT &ctxt, const DOF6 &tf)
{
  std::map<typename OBJECT::Ptr,bool> used;
  return ctxt_.merge(ctxt, tf, used, false);
}

template<typename OBJECT_CONTEXT>
bool Node<OBJECT_CONTEXT>::merge(const OBJCTXT &ctxt, DOF6 &tf, std::map<typename OBJECT::Ptr,bool> &used, const bool only_merge)
{
  //1. find correspondences
  //2. register
  return ctxt_.merge(ctxt, tf, used, only_merge);
}

template<typename OBJECT_CONTEXT>
bool Node<OBJECT_CONTEXT>::compute(const OBJCTXT &ctxt, DOF6 &link, std::map<typename OBJECT::Ptr, bool> &used, const bool only_merge)
{
  //check bounding box
  if(!(connections_.size()==0 && ctxt_.empty()) && !(ctxt_.getBoundingBox().transform(link.getRotation(),link.getTranslation())&ctxt.getBoundingBox().transform(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero())))
  {
    ROS_INFO("no intersection");
    return false;
  }

  std::cout<<"COMPUTING\n"<<link<<"\n";
  for(size_t i=0; i<connections_.size(); i++)
    std::cout<<"LINK BEF"<<connections_[i].link_<<"\n";

  DOF6 cp_link;
  cp_link.deepCopy(link);

  //register
  typename DOF6::TYPE success_rate, error_rate;
  bool r = registration(ctxt, link, success_rate, error_rate);
  if(!r)
  {
    // restore link
    link.deepCopy(cp_link);
    ROS_INFO("restore link");
  }
  bool ret = r || (connections_.size()==0 && ctxt_.empty());

  ROS_INFO("r was %d",r);
  std::cout<<link<<"\n";

  for(size_t i=0; i<connections_.size(); i++)
  {
    std::cout<<"LINK AFTER"<<connections_[i].link_<<"\n";
    std::cout<<"COMPUTING AFTER SHOULD"<<(typename DOF6::TROTATION)( ((Eigen::Matrix3f)link.getRotation())*((Eigen::Matrix3f)connections_[i].link_.getRotation()))<<"\n";

    DOF6 tmp_link;
    tmp_link.deepCopy(link);
    tmp_link.setVariance(link.getTranslationVariance()+connections_[i].link_.getTranslationVariance(),
                         ((Eigen::Matrix3f)link.getRotation())*connections_[i].link_.getTranslation()+link.getTranslation(),
                         link.getRotationVariance()+connections_[i].link_.getRotationVariance(),
                         (typename DOF6::TROTATION)( ((Eigen::Matrix3f)link.getRotation())*((Eigen::Matrix3f)connections_[i].link_.getRotation())) );
    tmp_link.getSource1()->reset();
    bool r2 = connections_[i].node_->compute(ctxt, tmp_link, used, true);

    ROS_INFO("r2 was %d",r2);

    if(r2&&r) {
      *connections_[i].link_.getSource1() += link.getSource1()->transpose() + *tmp_link.getSource1(); //TODO: is not correct
    }
    else if(r2) {
      *link.getSource1() += connections_[i].link_.getSource1()->transpose() + *tmp_link.getSource1(); //TODO: is not correct
      std::cout<<"CONNECTION\n";
      //std::cout<<*connections_[i].link_.getSource1()<<"\n";
      std::cout<<connections_[i].link_<<"\n";
      std::cout<<"TMPLINK\n";
      std::cout<<*tmp_link.getSource1()<<"\n";
      std::cout<<"LINK\n";
      std::cout<<*link.getSource1()<<"\n";
      std::cout<<"SHOULD\n";
      std::cout<<(connections_[i].link_.getSource1()->getRotation().inverse()*tmp_link.getSource1()->getRotation())<<"\n";
      ret = true;
    }
  }

  if(!ret && connections_.size()>0)
  {
    ROS_INFO("need a complete map");

    OBJCTXT map;
    map += ctxt_;
    for(size_t i=0; i<connections_.size(); i++)
    {
      map += connections_[i].node_->ctxt_.clone()->transform(connections_[i].link_);
    }
    ret = map.registration(ctxt, link, success_rate, error_rate);

    if(ret)
      ROS_INFO("worked :)");
    else
      ROS_INFO("did not work :(");
  }

  if(ret)
    merge(ctxt, link, used, only_merge);

  ctxt_.update();

  return ret;
}


