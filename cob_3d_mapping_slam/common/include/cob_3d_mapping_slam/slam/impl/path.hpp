
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
  act_ctxt_.update();

  //registration
  //typename DOF6::TYPE success_rate, error_rate;
  //local_.node_->registration(act_ctxt_, local_.link_, success_rate, error_rate);
  {
    std::map<typename OBJECT::Ptr,bool> used;
    local_.node_->compute(act_ctxt_, local_.link_, used);
  }

  if(needNewNode()) {
    // new node needed
    ROS_INFO("new node");

    //store node
    path_.push_back(local_);

    //create new node
    newNode();
    local_.link_.setTime(last_time_);

    //add path to new node (straight-forwared)
    local_.node_->addLink(path_.back());

    //our new node gets some object (yummy)
    //local_.node_->addCtxt(act_ctxt_, local_.link_);
  }
  else
  {
    ROS_INFO("no new node");

    //local_.node_->addCtxt(act_ctxt_, local_.link_);
  }

}
