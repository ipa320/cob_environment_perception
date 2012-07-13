
template<typename KEY, typename NODE>
void Context<KEY,NODE>::startFrame(const double time_in_sec)
{
  path_.startFrame(time_in_sec);
}

template<typename KEY, typename NODE>
void Context<KEY,NODE>::operator+=(typename OBJECT::Ptr obj)
{
  path_+=obj;

  KEY key(obj);
  typename KEY::KEYS keys = key.getKeys();
  for(typename KEY::KEYS::const_iterator k = keys.begin(); k!=keys.end(); k++)
  {
    typename MAP::iterator it = map_.find(*k);
  }
}

template<typename KEY, typename NODE>
void Context<KEY,NODE>::finishFrame()
{
  path_.finishFrame();
}

