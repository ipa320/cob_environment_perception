#pragma once

namespace Eigen {
	

namespace internal {

#ifndef EIGEN_PARSED_BY_DOXYGEN
template<typename BVH, typename Intersector>
bool intersect_helperx(const BVH &tree, Intersector &intersector, typename BVH::Index root)
{
  typedef typename BVH::Index Index;
  typedef typename BVH::VolumeIterator VolIter;
  typedef typename BVH::ObjectIterator ObjIter;

  bool found = false;
  VolIter vBegin = VolIter(), vEnd = VolIter();
  ObjIter oBegin = ObjIter(), oEnd = ObjIter();

  std::vector<Index> todo(1, root);

  while(!todo.empty()) {
    tree.getChildren(todo.back(), vBegin, vEnd, oBegin, oEnd);
    todo.pop_back();

    for(; vBegin != vEnd; ++vBegin) //go through child volumes
      if(intersector.intersectVolume(tree.getVolume(*vBegin)))
        todo.push_back(*vBegin);

    for(; oBegin != oEnd; ++oBegin) //go through child objects
      if(intersector.intersectObject(*oBegin)) {
        found = true;
        if(intersector.add(*oBegin)) return found;
	  }
  }
  return found;
}
#endif //not EIGEN_PARSED_BY_DOXYGEN

} // end namespace internal

/**  Given a BVH, runs the query encapsulated by \a intersector.
  *  The Intersector type must provide the following members: \code
     bool intersectVolume(const BVH::Volume &volume) //returns true if volume intersects the query
     bool intersectObject(const BVH::Object &object) //returns true if the search should terminate immediately
  \endcode
  */
template<typename BVH, typename Intersector>
void BVIntersect2(const BVH &tree, Intersector &intersector)
{
  internal::intersect_helperx(tree, intersector, tree.getRootIndex());
}


}
