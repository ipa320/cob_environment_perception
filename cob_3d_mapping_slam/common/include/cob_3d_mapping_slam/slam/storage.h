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
/*
 * storage.h
 *
 *  Created on: 11.06.2012
 *      Author: josh
 */

#ifndef STORAGE_H_
#define STORAGE_H_


namespace Slam  /// namespace for all Slam related stuff
{

  /**
   * storage helper
   *
   * list of used nodes:
   *    A -> B -> C -> ...
   *
   * update a node if
   *  - similiar object was found in object context of node
   *  - a connection to this node was active
   *
   * update is:
   *  - move it to the beginning
   *  - update sequence number
   *
   * go from end to front while true:
   *  - if actual sequence number minus sequence number of node is bigger than threshold
   *    -> save node to hard disk
   *    -> remove node from ram (keep links to object context)
   */
  template<typename NODE>
  class Storage
  {

    int seq_nr_, seq_thr_;

    struct STORAGE
    {
      typename NODE::Ptr node_;
      int seq_nr_;
    };

    std::list<STORAGE> used_list_;

  public:

    Storage(const int thr):
      seq_nr_(0),
      seq_thr_(thr)
    {

    }

    void update(typename NODE::Ptr node)
    {
      for(std::list<STORAGE>::iterator it=used_list_.begin(); it!=used_list_.end(); it++)
      {
        if(it->node_ == node)
        {
          it->seq_nr_ = seq_nr_;
          used_list_.push_back(*it);
          used_list_.remove(it);
          return;
        }
      }

      Storage S;
      S.seq_nr_ = seq_nr_;
      S.node_ = node;

      used_list_.push_back(S);
    }

    void finish()
    {
      ++seq_nr_;
      for(std::list<STORAGE>::iterator it=used_list_.begin(); it!=used_list_.end(); it++)
      {
        if(seq_nr_-it->seq_nr_>seq_thr_)
        {
          it->node_->store();
          it->node_->remove();
        }
        else
          return;
      }
    }

  };

};


#endif /* STORAGE_H_ */
