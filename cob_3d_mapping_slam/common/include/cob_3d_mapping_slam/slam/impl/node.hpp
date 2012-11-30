/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


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
bool Node<OBJECT_CONTEXT>::compute(const OBJCTXT &ctxt, DOF6 &link, std::map<typename OBJECT::Ptr, bool> &used, const bool only_merge, const int depth)
{
  //check bounding box
  if(depth>1 && !(connections_.size()==0 && ctxt_.empty()) && !(ctxt_.getBoundingBox().transform(link.getRotation(),link.getTranslation())&ctxt.getBoundingBox().transform(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero())))
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
    bool r2 = connections_[i].node_->compute(ctxt, tmp_link, used, true, depth+1);

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


