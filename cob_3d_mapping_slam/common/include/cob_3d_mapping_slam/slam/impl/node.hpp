
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
bool Node<OBJECT_CONTEXT>::merge(const OBJCTXT &ctxt, const DOF6 &tf, std::map<typename OBJECT::Ptr,bool> &used, const BoundingBox::TransformedFoVBB &fov, const bool only_merge)
{
  //1. find correspondences
  //2. register
  return ctxt_.merge(ctxt, tf, used, fov, only_merge);
}

template<typename OBJECT_CONTEXT>
bool Node<OBJECT_CONTEXT>::compute(const OBJCTXT &ctxt, DOF6 &link, std::map<typename OBJECT::Ptr, bool> &used, std::map<const Node*,bool> &_visited_list_, const bool only_merge, const int depth)
{
  _visited_list_.clear();
  if(_register(ctxt,link,used,_visited_list_, only_merge, depth) || (connections_.size()==0 && ctxt_.empty())) {
    _visited_list_.clear();
    _merge(ctxt,link,used,_visited_list_, only_merge, depth);
  }
}


template<typename OBJECT_CONTEXT>
bool Node<OBJECT_CONTEXT>::_register(const OBJCTXT &ctxt, DOF6 &link, std::map<typename OBJECT::Ptr, bool> &used, std::map<const Node*,bool> &_visited_list_, const bool only_merge, const int depth)
{
  //prevent endless loop
  if(_visited_list_.find(this)!=_visited_list_.end())
    return false;
  _visited_list_[this] = true;

  const float noise = 0.02f;

  //check bounding box
  if(depth>1 && !(connections_.size()==0 && ctxt_.empty()) && !(ctxt_.getBoundingBox().transform(link.getRotation(),link.getTranslation())&ctxt.getBoundingBox().transform(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero())))
  {
    ROS_INFO("no intersection");
    return false;
  }

  std::cout<<"COMPUTING\n"<<link<<"\n";
//  for(size_t i=0; i<connections_.size(); i++)
//    std::cout<<"LINK BEF"<<connections_[i].link_<<"\n";

  DOF6 cp_link;
  cp_link.deepCopy(link);

  //register
  typename DOF6::TYPE success_rate, error_rate;
  bool r = registration(ctxt, link, success_rate, error_rate);
  if(!r)
  {
    // restore link
    link.deepCopy(cp_link);
    link.getSource1()->reset();
    ROS_INFO("restore link");
  }
  bool ret = r || (connections_.size()==0 && ctxt_.empty());

  ROS_INFO("r was %d",r);
  std::cout<<link<<"\n";

  for(size_t i=0; i<connections_.size(); i++)
  {
    //    std::cout<<"LINK AFTER\n"<<connections_[i].link_<<"\n";
    //    std::cout<<"COPY LINK\n"<<cp_link<<"\n";

    //#error cp_link variance different from bef.
    DOF6 tmp_link;
    tmp_link.deepCopy(link);
    tmp_link.setVariance(cp_link.getTranslationVariance()+connections_[i].link_.getTranslationVariance() + noise,
                         ((Eigen::Matrix3f)link.getRotation())*connections_[i].link_.getTranslation()+link.getTranslation(),
                         cp_link.getRotationVariance()+connections_[i].link_.getRotationVariance() + noise,
                         (typename DOF6::TROTATION)( ((Eigen::Matrix3f)link.getRotation())*((Eigen::Matrix3f)connections_[i].link_.getRotation())) );
    tmp_link.getSource1()->reset();
    bool r2 = connections_[i].node_->_register(ctxt, tmp_link, used, _visited_list_, true, depth+1);

    ROS_INFO("r2 was %d",r2);

    if(r2&&r) {
      std::cout<<"LINK BEF\n";
      std::cout<<*connections_[i].link_.getSource1()<<"\n";

      *connections_[i].link_.getSource1() += link.getSource1()->transpose() + *tmp_link.getSource1(); //TODO: is not correct
      connections_[i].link_.setVariance(connections_[i].link_.getTranslationVariance(), connections_[i].link_.getTranslation(),
                                        connections_[i].link_.getRotationVariance(), connections_[i].link_.getRotation());

      std::cout<<"UPDATE LINK\n";;

      std::cout<<"CONNECTION\n";;
      std::cout<<link<<"\n";
      std::cout<<"CONNECTION TRANSPOSED\n";;
      std::cout<<link.getSource1()->transpose()<<"\n";
      std::cout<<"TMPLINK\n";
      std::cout<<*tmp_link.getSource1()<<"\n";
      std::cout<<"ADDED\n";
      std::cout<<(link.getSource1()->transpose() + *tmp_link.getSource1())<<"\n";

      std::cout<<"LINK\n";
      std::cout<<*connections_[i].link_.getSource1()<<"\n";
      std::cout<<"SHOULD\n";
      std::cout<<(link.getSource1()->getRotation().inverse()*tmp_link.getSource1()->getRotation())<<"\n";

      //ret = (link.getSource1()->getTranslationVariance()+link.getSource1()->getRotationVariance())<0.3f;
    }
    else if(r2 /*&& (connections_[i].link_.getSource1()->getTranslationVariance()+connections_[i].link_.getSource1()->getRotationVariance())<0.3f*/ ) {
      if(connections_[i].link_.getSource1()->isSet())
      {
        ROS_INFO("rX was 1");
        *link.getSource1() += connections_[i].link_.getSource1()->transpose() + *tmp_link.getSource1(); //TODO: is not correct
        link.setVariance(link.getTranslationVariance(), link.getTranslation(),
                         link.getRotationVariance(), link.getRotation());

        std::cout<<"CONNECTION\n";;
        std::cout<<connections_[i].link_<<"\n";
        std::cout<<"CONNECTION TRANSPOSED\n";;
        std::cout<<connections_[i].link_.getSource1()->transpose()<<"\n";
        std::cout<<"TMPLINK\n";
        std::cout<<tmp_link<<"\n";
        std::cout<<"ADDED\n";
        std::cout<<(connections_[i].link_.getSource1()->transpose() + *tmp_link.getSource1())<<"\n";

        std::cout<<"LINK\n";
        std::cout<<link<<"\n";
        std::cout<<"SHOULD\n";
        std::cout<<(connections_[i].link_.getSource1()->getRotation().inverse()*tmp_link.getSource1()->getRotation())<<"\n";
      }
      else
      {
        ROS_INFO("rX was 2");
        //TODO:
        Eigen::Matrix4f M = connections_[i].link_.getTF4().inverse();
        Eigen::Matrix4f M2= M*tmp_link.getTF4();
        link.setVariance(tmp_link.getTranslationVariance()+connections_[i].link_.getTranslationVariance(),
                         M2.col(3).head<3>(),
                         tmp_link.getRotationVariance()+connections_[i].link_.getRotationVariance(),
                         (typename DOF6::TROTATION)( M2.topLeftCorner(3,3) ) );

        std::cout<<"CONNECTION\n";;
        std::cout<<connections_[i].link_<<"\n";
        std::cout<<"CONNECTION\n";;
        std::cout<<connections_[i].link_.getTF4()<<"\n";
        std::cout<<"CONNECTION TRANSPOSED\n";;
        std::cout<<M<<"\n";
        std::cout<<"TMPLINK\n";
        std::cout<<tmp_link<<"\n";
        std::cout<<"LINK\n";
        std::cout<<link<<"\n";
      }

      ret |= (link.getTranslationVariance()+link.getRotationVariance())<0.5f;

      ROS_INFO("sumvar %f",(link.getTranslationVariance()+link.getRotationVariance()));
    }
    else
      ROS_INFO("au backe");

  }

  ROS_INFO("ret was %d",ret);

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
    {
      ROS_INFO("did not work :(");

      // restore link
      link.deepCopy(cp_link);
      ROS_INFO("restore link");
    }
  }

  return ret;
}

template<typename OBJECT_CONTEXT>
void Node<OBJECT_CONTEXT>::_merge(const OBJCTXT &ctxt, const DOF6 &link, std::map<typename OBJECT::Ptr, bool> &used, std::map<const Node*,bool> &_visited_list_, const bool only_merge, const int depth)
{
  //prevent endless loop
  if(_visited_list_.find(this)!=_visited_list_.end())
    return;
  _visited_list_[this] = true;

  const float noise = 0.06f;

  //check bounding box
  if(depth>1 && !(connections_.size()==0 && ctxt_.empty()) && !(ctxt_.getBoundingBox().transform(link.getRotation(),link.getTranslation())&ctxt.getBoundingBox().transform(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero())))
  {
    ROS_INFO("no intersection");
    return;
  }

  for(size_t i=0; i<connections_.size(); i++)
  {
    DOF6 tmp_link;
    tmp_link.deepCopy(link);
    tmp_link.setVariance(link.getTranslationVariance()+connections_[i].link_.getTranslationVariance() + noise,
                         ((Eigen::Matrix3f)link.getRotation())*connections_[i].link_.getTranslation()+link.getTranslation(),
                         link.getRotationVariance()+connections_[i].link_.getRotationVariance() + noise,
                         (typename DOF6::TROTATION)( ((Eigen::Matrix3f)link.getRotation())*((Eigen::Matrix3f)connections_[i].link_.getRotation())) );
    tmp_link.getSource1()->reset();
    connections_[i].node_->_merge(ctxt, tmp_link, used, _visited_list_, true, depth+1);
  }

  BoundingBox::FoVBB<float> fov;
  Eigen::Vector3f mi,ma;
  mi(0)=-0.43f;
  mi(1)=-0.36f;
  ma(0)=0.43f;
  ma(1)=0.36f;
  mi(2)=ma(2)=1.f;
  mi*=0.4f;
  ma*=8.f;
  Eigen::Vector3f t;
  t(0)=t(1)=0;
  t(2)=0.4f;
  fov.update(mi,ma);
  DOF6 l=link.transpose();
  merge(ctxt, link, used, fov.transform(l.getRotation(),l.getTranslation()-t), only_merge);

  ctxt_.update();
}


