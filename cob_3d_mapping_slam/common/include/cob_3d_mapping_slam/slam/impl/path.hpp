
template<typename NODE>
void Path<NODE>::startFrame(const double time_in_sec)
{
  act_ctxt_.clear();

  local_.link_.setTime(time_in_sec);
  last_time_ = time_in_sec;
}

template<typename NODE>
void Path<NODE>::operator+=(typename OBJECT::Ptr obj)
{
  act_ctxt_ += obj;

  //KEY key(obj);
  //std::map<KEY,typename NODE::Ptr>::iterator it = map_.find(key);
}

template<typename NODE>
void Path<NODE>::finishFrame()
{

  //registration
  //typename DOF6::TYPE success_rate, error_rate;
  //local_.node_->registration(act_ctxt_, local_.link_, success_rate, error_rate);
  {
    std::map<typename OBJECT::Ptr,bool> used;
    std::map<const NODE*,bool> visited_list;
    std::cout<<"LINK BEF123\n"<<local_.link_;
    local_.node_->compute(act_ctxt_, local_.link_, used, visited_list);
    std::cout<<"LINK AFT123\n"<<local_.link_;
  }

  if(needNewNode()) {
    const SWAY<NODE> * add = NULL;
    DOF6 l;
    //check if we can reuse existing node
    for(size_t i=0; i<local_.node_->getConnections().size(); i++)
    {
      //TODO: take nearest

      //get updated tf
      if(!getTF(l, &local_.node_->getConnections()[i], &local_))
        continue;

      std::cout<<"CHECKING LINK\n"<<l<<"\n";

      l = l+local_.link_;

      std::cout<<"CHECKING LINK\n"<<l<<"\n";

      if(
          //translation
          l.getTranslation().norm() + l.getTranslationVariance() < translation_res_
          &&
          //rotation
          l.getRotation().norm() + l.getRotationVariance() < rotation_res_
      )
      {
        add = &local_.node_->getConnections()[i];
        break;
      }
    }

    if(add)
    {
      // reuse node
      ROS_INFO("reuse node");

      //store node
      path_.push_back(local_);

      std::cout<<"REUSED LINK\n"<<l<<"\n";

      local_ = *add;
      local_.link_.deepCopy(l);
//      *local_.link_.getSource1() = *path_.back().link_.getSource1() + *l.getSource1();
//      local_.link_.setVariance(local_.link_.getSource1()->getTranslationVariance(),
//                           local_.link_.getSource1()->getTranslation(),
//                           local_.link_.getSource1()->getRotationVariance(),
//                           (typename DOF6::TROTATION)( local_.link_.getSource1()->getRotation()) );

      //add path to new node (straight-forwared)
      local_.node_->addLink(path_.back());
    }
    else
    {
      // new node needed

      //store node
      path_.push_back(local_);

      //create new node
      newNode();
      local_.link_.setTime(last_time_);

      //add path to new node (straight-forwared)
      local_.node_->addLink(path_.back());

      //our new node gets some object (yummy)
      //local_.node_->addCtxt(act_ctxt_, local_.link_);

      ROS_INFO("new node %d", local_.id_);
    }
  }
  else
  {
    ROS_INFO("no new node");

    //local_.node_->addCtxt(act_ctxt_, local_.link_);
  }

}

template<typename NODE>
bool Path<NODE>::getTF(DOF6 &tf, const SWAY<NODE> *start, const SWAY<NODE> *end)
{
  if(start->id_<end->id_)
  {
    bool r = getTF(tf,end,start);
    tf.transpose();
    return r;
  }

  ROS_INFO("IDSSSS %d %d", start->id_, end->id_);

  if(start->id_!=end->id_)
  {
    for(size_t i=0; i<start->node_->getConnections().size(); i++)
    {
      if(start->node_->getConnections()[i].id_==end->id_)
      {
        tf.deepCopy(start->node_->getConnections()[i].link_);
        std::cout<<"SET LINK\n"<<tf<<"\n";
        return true;
      }
      else if(getTF(tf, &start->node_->getConnections()[i], end)) {    //TODO: takes only first
        tf = tf + start->node_->getConnections()[i].link_;
        std::cout<<"ADD LINK\n"<<start->node_->getConnections()[i].link_<<"\n";
        std::cout<<"NEW LINK\n"<<tf<<"\n";
        return true;
      }
    }
  }

  return false;
}
