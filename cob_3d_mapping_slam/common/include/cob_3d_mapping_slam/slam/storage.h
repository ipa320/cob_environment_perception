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
