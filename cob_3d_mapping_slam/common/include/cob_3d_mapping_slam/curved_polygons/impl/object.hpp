
template<typename _DOF6>
bool Object<_DOF6>::operator|(const Object &o) const
{

  //check overlap

#if 0
  //first bb
  //ROS_INFO("check %d %d %d",  data_.intersectsBB(o.data_, 0.4 /*metres*/), data_.fitsCurvature(o.data_, 0.1 /*rad*/), data_.extensionMatch(o.data_, 0.2 /*percent*/));
  if( data_.intersectsBB(o.data_, 0.01 /*metres*/)&&data_.extensionMatch(o.data_, 0.05 /*percent*/) )
  {
    return true;
  }
#endif

#if 1
  if( (data_.getNearestPoint()-o.data_.getNearestPoint()).squaredNorm() < 0.1f*0.1f ||
      (data_.getFeatures()[2].v_-o.data_.getFeatures()[2].v_).squaredNorm() < 0.1f*0.1f )
  {
    return true;
  }
#endif

  return false;
}

template<typename _DOF6>
bool Object<_DOF6>::isReachable(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const
{
//  if( (!data_.extensionMatch(o.data_, 0.25f /*percent*/, 0.1f) && !data_.matchForm(o.data_)) )//||
//      //data_.isPlane() != o.data_.isPlane() /*||!data_.fitsCurvature(o.data_, 0.3)*/)
//    return false;

  Eigen::Matrix<typename TFLINK::TYPE, 3,1> v1,v2;

  for(size_t i=0; i<data_.getFeatures().size(); i++) {
    if(data_.getFeatures()[i].type_ != ex_curved_polygon::S_FEATURE::POINT && data_.getFeatures()[i].type_ != ex_curved_polygon::S_FEATURE::NORMAL) continue;
    v1 = data_.getFeatures()[i].v_;
    v2 = o.data_.getFeatures()[i].v_;

    if(!(( (v2-v1).squaredNorm()>thr_tr*thr_tr &&
        std::acos(v2.dot(v1)/(v2.norm()*v1.norm())) > thr_rot) ||
        std::abs(v2.norm()-v1.norm()) > thr_tr + 0.02*v2.norm())
    )
    {
      return true;
    }
  }

  return false;
}

template<typename _DOF6>
std::vector<typename _DOF6::TYPE> Object<_DOF6>::getDistance(const Object &o) const
{
  std::vector<typename DOF6::TYPE> r;

  for(size_t i=0; i<data_.getFeatures().size(); i++)
  {
    if(data_.getFeatures()[i].type_ != ex_curved_polygon::S_FEATURE::POINT) continue;
    for(size_t j=0; j<o.data_.getFeatures().size(); j++)
    {
      if(o.data_.getFeatures()[j].type_ != ex_curved_polygon::S_FEATURE::POINT || o.data_.getFeatures()[j].ID_!=data_.getFeatures()[i].ID_) continue;

      Eigen::Matrix<typename TFLINK::TYPE, 3,1> v1,v2;

      v1 = data_.getFeatures()[i].v_;
      v2 = o.data_.getFeatures()[j].v_;

      r.push_back((v1-v2).norm());
    }
  }

  return r;
}

template<typename _DOF6>
bool Object<_DOF6>::operator&(const Object &o) const
{

  if(data_.intersectsBB(o.data_, 0.05 /*metres*/) || data_.fitsCurvature(o.data_, 0.1 /*rad*/))
    return true;

  Eigen::Matrix<typename TFLINK::TYPE, 3,1> v1,v2;

  for(size_t i=0; i<data_.getFeatures().size(); i++) {
    if(data_.getFeatures()[i].type_ != ex_curved_polygon::S_FEATURE::POINT) continue;
    v1 = data_.getFeatures()[i].v_;
    v2 = o.data_.getFeatures()[i].v_;

    if(!(( (v2-v1).squaredNorm()>0.02*0.02 &&
        std::acos(v2.dot(v1)/(v2.norm()*v1.norm())) > 0.02) ||
        std::abs(v2.norm()-v1.norm()) > 0.02 + 0.02*v2.norm())
    )
    {
      return true;
    }
  }

  //validate extensions

  //return (*this)|o;

  if( std::min(data_.getFeatures()[1].v_org_.squaredNorm(),o.data_.getFeatures()[1].v_org_.squaredNorm())
  /std::max(data_.getFeatures()[1].v_org_.squaredNorm(),o.data_.getFeatures()[1].v_org_.squaredNorm()) < 0.6f )
    return false;

  //check overlap

#if 1
  //first bb
  //ROS_INFO("check %d %d %d",  data_.intersectsBB(o.data_, 0.4 /*metres*/), data_.fitsCurvature(o.data_, 0.1 /*rad*/), data_.extensionMatch(o.data_, 0.2 /*percent*/));
  if( //data_.extensionMatch(o.data_, 0.2 /*percent*/) &&
      //data_.intersectsBB(o.data_, 0.05 /*metres*/) &&
      (data_.matchForm(o.data_)
          //     ||
          //     ( (*this|o) && data_.extensionMatch(o.data_, 0.2 /*percent*/) && data_.intersectsBB(o.data_, 0.05 /*metres*/))
      ) &&
      data_.isPlane() == o.data_.isPlane()
      //data_.fitsCurvature(o.data_, 0.15 /*rad*/)
      //      ||
      //(data_.intersectsBB(o.data_, 0.05 /*metres*/) && data_.fitsCurvature(o.data_, 0.15 /*rad*/))
      //||
      //      (*this|o)
      //&& data_.extensionMatch(o.data_, 0.1 /*percent*/)
  )
  {
    data_.printEnergy();
    o.data_.printEnergy();
    return true;
  }
#endif

#if 0
  //check wether we are similiar

  //wether A to B
  if(o.data_.getID() == data_.getID())
    return true;

  /*for(size_t i=0; i<data_.getScore().size(); i++)
  {

    if(o.data_.getID() == data_.getScore()[i].ID)
    {
      return true;
    }

    //or B to A
    for(size_t j=0; j<o.data_.getScore().size(); j++)
    {

      if(data_.getID() == o.data_.getScore()[j].ID  || o.data_.getScore()[j].ID == data_.getScore()[i].ID)
      {
        return true;
      }
    }

  }*/
#endif

  return false;//no match :(
}


template<typename _DOF6>
typename Object<_DOF6>::TFLIST Object<_DOF6>::getTFList(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr) const
{

  typename Object<_DOF6>::TFLIST list;
  typedef typename Object<_DOF6>::TF_CORS CORS;

  //  if(data_.isPlane() != o.data_.isPlane())
  //  {
  //    ROS_WARN("plane not plane...");
  //    //return list;
  //  }

  const float fw = data_.matchFormf(o.data_);
  ROS_INFO("fw %f",fw);
  if(fw<=0.) return list;
  //const float wX = fw*sqrtf(data_.getWeight()*o.data_.getWeight())/1000 / (1+data_.getFeatures()[2].v_.squaredNorm());
  const float wX = fw*used_/(float)creation_*std::min(data_.getWeight(),o.data_.getWeight())/1000 / (1+data_.getFeatures()[2].v_.squaredNorm());
  ROS_INFO("w %f",wX);

  //  if( std::min(data_.getFeatures()[1].v_org_.squaredNorm(),o.data_.getFeatures()[1].v_org_.squaredNorm())
  //  /std::max(data_.getFeatures()[1].v_org_.squaredNorm(),o.data_.getFeatures()[1].v_org_.squaredNorm()) < 0.75f )
  //    return list;

  //build up our features

  //ROS_ASSERT(data_.getFeatures().size() == o.data_.getFeatures().size());

  for(size_t i=0; i<data_.getFeatures().size(); i++) {

    if(i>=data_.getFeatures().size() || i>=o.data_.getFeatures().size())
      break;

    //for(size_t j=0; j<o.data_.getFeatures().size(); j++)
    size_t j=i;
    {
      if(data_.getFeatures()[i].ID_ != o.data_.getFeatures()[j].ID_ || data_.getFeatures()[i].type_ != o.data_.getFeatures()[j].type_)
        continue;

      ROS_ASSERT(data_.getFeatures()[i].type_ == o.data_.getFeatures()[j].type_);

      Eigen::Matrix<typename TFLINK::TYPE, 3,1> v1,v2;
      float w = wX;

      v1 = data_.getFeatures()[i].v_;
      v2 = o.data_.getFeatures()[j].v_;
      //if(!data_.isPlane()) return list;
      if(!pcl_isfinite(v1.sum()) || !pcl_isfinite(v2.sum()) || !pcl_isfinite(w)
          //|| TODO: check against threshold
      )
      {
        if( !pcl_isfinite(w) )
          ROS_WARN("weight is infinite");
        continue;
      }

      //std::cout<<"match "<<data_.getFeatures()[i].ID<<"\n"<<v1<<"\n\n"<<v2<<"\n";

      ex_curved_polygon::S_FEATURE ft_temp = data_.getFeatures()[i];
      ft_temp.transform(rot,tr,0,0);
      Eigen::Vector3f v1p = ft_temp.v_;
      switch(data_.getFeatures()[i].ID_)
      {
        case 1:
          if(data_.isPlane() != o.data_.isPlane()) break;
          w*=5;
        case 3:
          w*=0.2f;
        case -1:
        {
          bool normal=false;

          if(v2.squaredNorm()>40.f || v1p.squaredNorm()>40.f )
          {
            normal=true;
          }

          if( v2.squaredNorm()<0.1f || v1p.squaredNorm()<0.1f ||
              //(v2 - v1p).squaredNorm() > (thr_tr+thr_rot*v2.norm())*(thr_tr+thr_rot*v1.norm())
              ( (v2-v1p).squaredNorm()>thr_tr*thr_tr &&
                  std::acos(v2.dot(v1p)/(v2.norm()*v1p.norm())) > thr_rot) ||
                  std::abs(v2.norm()-v1p.norm()) > thr_tr + 0.03*v2.norm()
          )
          {
            //          std::cout<<"Ma:\n"<<v1<<"\n";
            //          std::cout<<"Mb:\n"<<v2<<"\n";
            //          std::cout<<"Ma*:\n"<<v1p<<"\n";
            //          std::cout<<"Mb*:\n"<<o.data_.getNearestTransformedPoint(rot, tr)<<"\n";

//            std::cout<<"\t\tMa:\n"<<v1<<"\n";
//            std::cout<<"\t\tMb:\n"<<v2<<"\n";
//            std::cout<<"\t\tMa*:\n"<<v1p<<"\n";
//            std::cout<<"\t\tMb*:\n"<<o.data_.getNearestTransformedPoint(rot, tr)<<"\n";
//            std::cout<<"\t\tMa**:\n"<<data_.getNearestTransformedPoint(rot, Eigen::Vector3f::Zero())<<"\n";
//            std::cout<<"\t\tMb**:\n"<<o.data_.getNearestTransformedPoint(rot, Eigen::Vector3f::Zero())<<"\n";
//            std::cout<<"\tnormal:  "<<normal<<"\n";
//            std::cout<<"\tplane:  "<<data_.isPlane()<<"\n";
//            std::cout<<"\tthr:  "<<thr_tr<<thr_rot<<"\n";
//            std::cout<<"\tthr:  "<<
//                std::abs(v2.norm()-v1p.norm())<<"/"<<
//                std::acos(v2.dot(v1p)/(v2.norm()*v1p.norm()))<<"\n";

            //if(data_.getFeatures()[i].ID_!=3) return list;
            break;
          }

//          std::cout<<"\t\ta:\n"<<v1<<"\n";
//          std::cout<<"\t\tb:\n"<<v2<<"\n";
//          std::cout<<"\t\ta*:\n"<<v1p<<"\n";
//          std::cout<<"\t\tb*:\n"<<o.data_.getNearestTransformedPoint(rot, tr)<<"\n";
//          std::cout<<"\t\ta**:\n"<<data_.getNearestTransformedPoint(rot, Eigen::Vector3f::Zero())<<"\n";
//          std::cout<<"\t\tb**:\n"<<o.data_.getNearestTransformedPoint(rot, Eigen::Vector3f::Zero())<<"\n";
//          std::cout<<"\tnormal:  "<<normal<<"\n";
//          std::cout<<"\tplane:  "<<data_.isPlane()<<"\n";
//          std::cout<<"\tthr:  "<<thr_tr<<"/"<<thr_rot<<"\n";
//          std::cout<<"\tthr:  "<<
//              std::abs(v2.norm()-v1p.norm())<<"/"<<
//              std::acos(v2.dot(v1p/
//                               (v2.norm()*v1p.norm()) ))<<"\n";

          const float w2 = data_.getFeatures()[i].type_==ex_curved_polygon::S_FEATURE::NORMAL? 1.5f : 1.f;
//          if(data_.getFeatures()[i].ID_!=3)
//          {
            if(normal)
              list.push_back( CORS(
                  typename TFLINK::TFLinkObj(v1,true,true, w2*w, w2*w),
                  typename TFLINK::TFLinkObj(v2,true,true, w2*w, w2*w)
              ) );
            else
              list.push_back( CORS(
                  typename TFLINK::TFLinkObj(v1,data_.getFeatures()[i].type_==ex_curved_polygon::S_FEATURE::NORMAL,false, w2*w, w2*w),
                  typename TFLINK::TFLinkObj(v2,o.data_.getFeatures()[j].type_==ex_curved_polygon::S_FEATURE::NORMAL,false, w2*w, w2*w)
              ) );

            ROS_ASSERT(!normal || (list.back().a.translation_M_.sum()==0&&list.back().b.translation_M_.sum()==0) );
//          }
//          else {
//            list.push_back( CORS(
//                typename TFLINK::TFLinkObj(v1,false,false, w2*w, w2*w),
//                typename TFLINK::TFLinkObj(v2,false,false, w2*w, w2*w)
//            ) );
//          }

          //        //TODO: not everything is a plane!!!
          //        list.push_back( CORS(
          //            typename TFLINK::TFLinkObj(v1,data_.isPlane()),
          //            typename TFLINK::TFLinkObj(v2,o.data_.isPlane())
          //        ) );
        }
        break;

        case 2:
          if( std::min(data_.getFeatures()[i].v_org_.squaredNorm(),o.data_.getFeatures()[j].v_org_.squaredNorm())
          /std::max(data_.getFeatures()[i].v_org_.squaredNorm(),o.data_.getFeatures()[j].v_org_.squaredNorm()) < 0.6f )
            break;

        case 4:
        case 5:
          w*=0.2f;

//          std::cout<<"\t\tv1 ("<<data_.getFeatures()[i].ID_<<")\n"<<v1<<"\n";
//          std::cout<<"\t\tv2 ("<<data_.getFeatures()[i].ID_<<")\n"<<v2<<"\n";
//          std::cout<<"\t\tv1* ("<<data_.getFeatures()[i].ID_<<")\n"<<v1p<<"\n";

          if( std::acos(v1p.dot(v2)) > thr_rot )
          {
//            std::cout<<"ABOVE MISS   ABOVE MISS   ABOVE MISS\n";
            break;
          }

          list.push_back( CORS(
              typename TFLINK::TFLinkObj(v1,true,true, w, w),
              typename TFLINK::TFLinkObj(v2,true,true, w, w)
          ) );

          ROS_ASSERT( (list.back().a.translation_M_.sum()==0&&list.back().b.translation_M_.sum()==0) );

          break;

          //      case 3:
          //        //TODO: seems buggy
          //        break;
        default:
          ROS_ASSERT(0);
      }

    }

  }

  return list;
}
