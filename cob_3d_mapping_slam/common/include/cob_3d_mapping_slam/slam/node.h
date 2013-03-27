/*
 * node.h
 *
 *  Created on: 26.05.2012
 *      Author: josh
 */

#ifndef NODE_H_
#define NODE_H_


#include "way.h"
#include "../curved_polygons/bb.h"

namespace Slam
{

  /**
   * properties:
   *  - atom node is a seen object
   *  - non-atom node is a view-point
   *  - each atom node can be reached by max. 2 hops
   */
  //template<typename DATA, typename LINK>
  template<typename OBJECT_CONTEXT>
  class Node
  {
  public:
    typedef typename OBJECT_CONTEXT::DOF6 DOF6;
    typedef OBJECT_CONTEXT OBJCTXT;
    typedef typename OBJCTXT::OBJECT OBJECT;

    typedef boost::shared_ptr<Node> Ptr;

  private:

    std::vector<SWAY<Node> > connections_;      /// connections to other nodes
    OBJCTXT ctxt_;                              /// context includes all operators for objects (registration, correspondences,...) + objects

    bool _register(const OBJCTXT &ctxt, DOF6 &link, std::map<typename OBJECT::Ptr,bool> &used, std::map<const Node*,bool> &_visited_list_, const bool only_merge=false, const int depth=0);
    void _merge(const OBJCTXT &ctxt, const DOF6 &link, std::map<typename OBJECT::Ptr,bool> &used, std::map<const Node*,bool> &_visited_list_, const bool only_merge=false, const int depth=0);

  public:

    void addLink(const SWAY<Node> &con) {
      connections_.push_back(con);
    }

    bool registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate);
    bool merge(const OBJCTXT &ctxt, const DOF6 &tf, std::map<typename OBJECT::Ptr,bool> &used, const BoundingBox::TransformedFoVBB &fov, const bool only_merge);

    bool addCtxt(const OBJCTXT &ctxt, const DOF6 &tf);

    size_t getNumObjs() const {return ctxt_.getNumObjs();}

    const std::vector<SWAY<Node> > &getConnections() const {return connections_;}

    const OBJECT_CONTEXT &getContext() const {return ctxt_;}

    bool compute(const OBJCTXT &ctxt, DOF6 &link, std::map<typename OBJECT::Ptr,bool> &used, std::map<const Node*,bool> &_visited_list_, const bool only_merge=false, const int depth=0);
  };

#include "impl/node.hpp"

}


#endif /* NODE_H_ */
