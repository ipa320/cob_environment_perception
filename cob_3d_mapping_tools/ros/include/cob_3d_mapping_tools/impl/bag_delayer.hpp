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
#include <boost/program_options.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_broadcaster.h>

#include <cob_3d_mapping_tools/keyboard_console_listener.hpp>

namespace cob_3d_mapping_tools
{
  struct BAG_MODE
  {
    enum modes
    {
      BACKWARD, FORWARD
    };
  };

  template<typename MsgT>
  class bag_delayer
  {
  private:
    typedef KeyboardConsoleListener KCL;

  public:
    bag_delayer()
      : mode(BAG_MODE::FORWARD)
      , b_run(false)
      , p_bag(NULL)
      , p_view(NULL)
      , n_msg(0)
      {
        keys[0] = KCL::KEYS::F;
        keys[1] = KCL::KEYS::B;
        keys[2] = KCL::KEYS::SPACE;
        keys_size = 3;
      }

    ~bag_delayer()
      { deinit(); }

    bool init(int argc, char* argv[]);
    void deinit();
    void run();

  private:
    void interpret_input(char c_in);


    BAG_MODE::modes mode;
    bool b_run;
    std::string topic_;

    rosbag::Bag *p_bag;
    rosbag::View *p_view;
    tf::TransformBroadcaster *tf_broadcaster;
    std::list<rosbag::View::iterator> timeline;

    int n_msg;
    char keys[3];
    int keys_size;
  };

  template <typename MsgT> bool
  bag_delayer<MsgT>::init(int argc, char* argv[])
  {
    std::string bag_file;
    unsigned int delay;

    using namespace boost::program_options;
    options_description options("Options");
    options.add_options()
      ("help", "produce help message")
      ("in", value<std::string>(&bag_file), "input pcd file")
      ("delay,d", value<unsigned int>(&delay)->default_value(1), "delay * 0.1 sec")
      ("topic,t", value<std::string>(&topic_)->default_value("/registration/pc_aligned"), "message topic to delay")
      ;

    positional_options_description p_opt;
    p_opt.add("in", 1);
    variables_map vm;
    store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
    notify(vm);

    if (vm.count("help") || argc == 1)
    {
      std::cout << "Press Key:" << std::endl;
      std::cout << "[f]     to step forward one message" << std::endl;
      std::cout << "[b]     to step backward one message" << std::endl;
      std::cout << "[space] to pause/unpause bagfile iteration in previously used direction" << std::endl;
      std::cout << options << std::endl;
      return false;
    }
    if (bag_file == "")
    {
      std::cout << "no input file defined " << std::endl << options << std::endl;
      return false;
    }

    p_bag = new rosbag::Bag();
    p_view = new rosbag::View();
    tf_broadcaster = new tf::TransformBroadcaster();

    try { p_bag->open(bag_file, rosbag::bagmode::Read); }
    catch (rosbag::BagException) { std::cerr << "Error opening file " << bag_file << std::endl; return false; }
    p_view->addQuery(*p_bag, rosbag::TopicQuery(topic_));
    p_view->addQuery(*p_bag, rosbag::TypeQuery("tf/tfMessage"));

    rosbag::View::iterator vit = p_view->begin();
    while(vit!=p_view->end()) { timeline.push_back(vit); ++vit; }

    KCL::get().init(delay);
    return true;
  }

  template <typename MsgT> void
  bag_delayer<MsgT>::deinit()
  {
    KCL::get().reset();
    if(p_bag) { p_bag->close(); delete p_bag; }
    delete p_view;
    delete tf_broadcaster;
  }

  template <typename MsgT> void
  bag_delayer<MsgT>::run()
  {
    typename MsgT::ConstPtr last_msg;
    tf::tfMessage::ConstPtr last_tf;
    std::list<rosbag::View::iterator>::iterator tl_it = timeline.begin();
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<MsgT>(this->topic_,1);
    while(ros::ok())
    {
      if(!this->b_run) { interpret_input(KCL::get().waitForIt(keys, keys_size)); }
      else { interpret_input(KCL::get().spinOnce()); }

      while(last_msg==NULL)
      {
        if(tl_it == timeline.end()) { deinit(); exit(0); }

        last_tf = (*tl_it)->instantiate<tf::tfMessage>();
        if (last_tf != NULL)
        {
          tf_broadcaster->sendTransform(last_tf->transforms);
          ros::spinOnce();
        }
        last_msg = (*tl_it)->instantiate<MsgT>();
        if(mode==BAG_MODE::FORWARD) { ++tl_it; }
        else if(mode==BAG_MODE::BACKWARD && tl_it != timeline.begin()) { --tl_it; }
        else { deinit(); exit(0); }
      }
      if (mode==BAG_MODE::FORWARD) std::cout << "Msg no."<<++n_msg<<" : " << last_msg->header.stamp << std::endl;
      else if (mode==BAG_MODE::BACKWARD) std::cout << "Msg no."<<--n_msg<<" : " << last_msg->header.stamp << std::endl;
      pub.publish(last_msg);
      last_msg.reset();
      last_tf.reset();
    }
  }

  template <typename MsgT> void
  bag_delayer<MsgT>::interpret_input(char c_in)
  {
    //std::cout << "IN: "<<(int)c_in<<std::endl;
    switch (c_in)
    {
    case KCL::KEYS::SPACE:
      //std::cout << "you hit space" << std::endl;
      this->b_run = !this->b_run;
      break;
    case KCL::KEYS::B:
      this->mode = BAG_MODE::BACKWARD;
      break;
    case KCL::KEYS::F:
      this->mode = BAG_MODE::FORWARD;
      break;
    }
  }
}
