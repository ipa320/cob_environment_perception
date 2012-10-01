#include "../../dof/feature.h"



template<typename _DOF6>
float Object<_DOF6>::getSimilarity(const Object &o) const
{
  const float fw = data_.matchFormf(o.data_);

  const float l1 = data_.getBB().extension();
  const float l2 = o.data_.getBB().extension();

  return fw * std::min(l1,l2)/std::max(l1,l2);
}


template<typename _DOF6>
const Slam_CurvedPolygon::ex_curved_polygon::BB &Object<_DOF6>::getBB() const
{
  return data_.getBB();
}

template<typename _DOF6>
Slam_CurvedPolygon::ex_curved_polygon::BB Object<_DOF6>::getBB(const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const
{
  return getBB().increaseIfNec(thr_rot, thr_tr);
}

/**
 * checks intersection of two bounding boxes
 *
 */
template<typename _DOF6>
bool Object<_DOF6>::intersectsBB(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const
{
  return getBB()&o.getBB(thr_rot,thr_tr);
}

/**
 * checks intersection of bounding box and point
 *
 */
template<typename _DOF6>
bool Object<_DOF6>::intersectsBB(const Eigen::Vector3f &o, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const
{
  return getBB().transform(rot,tr)&(DATA::BB(o).increaseIfNec(thr_rot, thr_tr));
}
template<typename _DOF6>
bool Object<_DOF6>::intersectsBB(const Eigen::Vector3f &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const
{
  return getBB()&(DATA::BB(o).increaseIfNec(thr_rot, thr_tr));
}

template<typename _DOF6>
bool Object<_DOF6>::intersectsPts(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr) const
{
	for(size_t i=0; i<o.data_.getPoints3D().size(); i++)
		if(intersectsBB(o.data_.getPoints3D()[i], thr_rot, thr_tr))
			return true;
	return false;
}


template<typename _DOF6>
void Object<_DOF6>::addTFList(const Object &o, const typename DOF6::TYPE &thr_rot, const typename DOF6::TYPE &thr_tr, const Eigen::Matrix3f &rot, const Eigen::Vector3f &tr, typename Object<_DOF6>::TFLIST &list) const
{

  typedef typename Object<_DOF6>::TF_CORS CORS;

  //  if(data_.isPlane() != o.data_.isPlane())
  //  {
  //    ROS_WARN("plane not plane...");
  //    //return list;
  //  }

  const float fw = data_.matchFormf(o.data_);
  //ROS_INFO("fw %f",fw);
  //if(fw<=0.11f && std::min(data_.extension(),o.data_.extension())<0.3f ) return;

  ROS_ASSERT(used_<=creation_);
  float wX = fw*used_*sqrtf(std::min(data_.getWeight(),o.data_.getWeight())/100) / (1+data_.getNearestPoint().norm());

  float wG=0.f;
  for(size_t i=0; i<o.data_.getOutline().size(); i++) {
      wG += (o.data_.getOutline()[(i+1)%o.data_.getOutline().size()].head(2)-o.data_.getOutline()[i].head(2)).norm();
  }
  wX /= 2*wG;

  //build up our features

  //ROS_ASSERT(data_.getFeatures().size() == o.data_.getFeatures().size());

  DATA::SURFACE surf = data_.getSurface();
  const DATA::SURFACE &surf1 = data_.getSurface();
  const DATA::SURFACE &surf2 = o.data_.getSurface();
  surf.transform(rot,tr);

  //ROS_INFO("addTF %d",data_.getOutline().size());
  if(data_.getOutline().size()>100||o.data_.getOutline().size()>100) {
    ROS_ERROR("too big, skipping");
    return;
  }

  for(size_t i=0; i<o.data_.getOutline().size(); i++) {

    Eigen::Vector3f p2 = o.data_.getOutline()[i];
    Eigen::Vector3f v2 = surf2.project2world(p2.head<2>());

    if(!intersectsBB(v2, rot,tr, thr_rot,thr_tr+0.025f)) {
      continue;
    }

    const float w = wX * (
        (o.data_.getOutline()[(i+1)%o.data_.getOutline().size()].head(2)-p2.head<2>()).norm()
        + (o.data_.getOutline()[(i-1+o.data_.getOutline().size())%o.data_.getOutline().size()].head(2)-p2.head<2>()).norm()
        );

    Eigen::Vector2f np = surf.nextPoint(v2); //next point

    Eigen::Vector3f np3;
    np3.head<2>() = np;
    np3(2) = std::max(p2(2), o.data_.getOutline()[(i+1)%o.data_.getOutline().size()](2));
    Eigen::Vector3f p1 = data_.getOutline().nextPoint(np3);
    //p1.head<2>() = np;
    p1(2) = p2(2);

    ::DOF6::S_FEATURE f1(surf.project2world(p1.head<2>()), surf.normalAt(p1.head<2>()), false);
    ::DOF6::S_FEATURE f2(surf2.project2world(p2.head<2>()), surf2.normalAt(p2.head<2>()), false);

    if(!f1.isReachable(f2,thr_tr,thr_rot) ) {
      //Debug::Interface::get().addArrow(surf1.project2world(p1.head<2>()),f2.v_,100,255,0);
      continue;
    }

    if(!pcl_isfinite(p1.sum())) {
      ROS_ERROR("invalid");
      continue;
    }

//    std::cerr<<"p1\n"<<p1<<"\n";
    ROS_ASSERT(pcl_isfinite(p1.sum()));

    float w_pt = w * (0.15f+std::min(p2(2), p1(2)));
    //float w_n  = w*0.05f;

    f1.v_ = surf1.project2world(p1.head<2>());
    f1.n_ = surf1.normalAt(p1.head<2>());

    if(pcl_isfinite(f1.v_.sum())&&pcl_isfinite(f2.v_.sum()))
      list.push_back( CORS(
          typename TFLINK::TFLinkObj(f1.v_, false,false, w_pt, w_pt),
          typename TFLINK::TFLinkObj(f2.v_,false,false, w_pt, w_pt)
      ) );

//    if(pcl_isfinite(f1.n_.sum())&&pcl_isfinite(f2.n_.sum()))
//      list.push_back( CORS(
//          typename TFLINK::TFLinkObj(-f1.n_, true,true, w_n, w_n),
//          typename TFLINK::TFLinkObj(-f2.n_, true,true, w_n, w_n)
//      ) );

  }

}
