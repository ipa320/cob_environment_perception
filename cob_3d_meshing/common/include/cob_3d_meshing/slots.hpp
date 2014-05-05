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
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_meshing
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 09/2013
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#ifndef SLOTS_HPP
#define SLOTS_HPP

/*
 * some design classes and functions for templated property handling
 * "Slots" is a wrapper class tieing together a variable list of properties
 * advantage: no v-table lookups for base-function calls
 */

class NullT { };

//_________________________________ Extensions ________________________________
template<typename PluginT, typename Base>
class Extension : public Base
{
private:
  PluginT plugin;

public:
  Extension(const PluginT& p, const Base& b)
    : Base(b), plugin(p) { }

  // for static policy
  template<typename PolicyT, typename ArgT>
  inline void run(const ArgT& args)
  {
    PolicyT::runPlugin(plugin,args); // call static function
    Base::template run<PolicyT,ArgT>(args); // recursive
  }

  // for non-static policy
  template<typename PolicyT, typename ArgT>
  inline void run(const PolicyT& policy, const ArgT& args)
  {
    policy(plugin,args); // call operator()
    Base::template run<PolicyT,ArgT>(policy, args);  // recursive
  }
};

//_____ specialization (end of recursive inheritance) _____
template<typename PluginT>
class Extension<PluginT, NullT>
{
private:
  PluginT plugin;

public:
  Extension(const PluginT& p, const NullT& b = NullT())
    : plugin(p) { }

  // for static policy
  template<typename PolicyT, typename ArgT>
  inline void run(const ArgT& args) { PolicyT::runPlugin(plugin,args); }

  // for non-static policy
  template<typename PolicyT, typename ArgT>
  inline void run(const PolicyT& policy, const ArgT& args) { policy(plugin,args); }
};

//_____ specialization (null type) _____
template<>
class Extension<NullT, NullT>
{
public:
  Extension(const NullT& p = NullT(), const NullT& b = NullT())
  { }

  template<typename PolicyT, typename ArgT>
  inline void run(const ArgT& args) { return; }

  template<typename PolicyT, typename ArgT>
  inline void run(const PolicyT& obj, const ArgT& args) { return; }
};


//_________________________________ Slots _____________________________________
template<typename P1 = NullT,
         typename P2 = NullT,
         typename P3 = NullT,
         typename P4 = NullT,
         typename P5 = NullT>
class Slots : public Extension<P1, typename Slots<P2,P3,P4,P5,NullT>::BaseT>
{
public:
  typedef Slots<P2,P3,P4,P5,NullT> BaseSlots;
  typedef Extension<P1, typename BaseSlots::BaseT> BaseT;

  Slots(const P1& plugin1 = P1(),
        const P2& plugin2 = P2(),
        const P3& plugin3 = P3(),
        const P4& plugin4 = P4(),
        const P5& plugin5 = P5())
    : BaseT( plugin1, BaseSlots(plugin2, plugin3, plugin4, plugin5, NullT()) )
  { }

  // for static policy
  template<typename PolicyT, typename ArgT>
  inline void run(const ArgT& args)
  {
    BaseT::template run<PolicyT,ArgT>(args);
  }

  // for non-static policy
  template<typename PolicyT, typename ArgT>
  inline void run(const PolicyT& policy, const ArgT& args)
  {
    BaseT::template run<PolicyT,ArgT>(policy, args);
  }
};

//_____ specialization _____
template<typename P1>
class Slots<P1, NullT, NullT, NullT, NullT> : public Extension<P1, NullT>
{
public:
  typedef Extension<P1, NullT> BaseT;

  Slots(const P1&    plugin1 = P1(),
        const NullT& plugin2 = NullT(),
        const NullT& plugin3 = NullT(),
        const NullT& plugin4 = NullT(),
        const NullT& plugin5 = NullT())
    : BaseT(plugin1) { }

  template<typename PolicyT, typename ArgT>
  inline void run(const ArgT& args)
  {
    BaseT::template run<PolicyT,ArgT>(args);
  }

  template<typename PolicyT, typename ArgT>
  inline void run(const PolicyT& policy, const ArgT& args)
  {
    BaseT::template run<PolicyT,ArgT>(policy, args);
  }
};

//_________________ convenience functions _____________________________________

// run specific policy on slots with arguemnts for static policy class
template<typename Policy, typename Args, typename Slots>
inline void runPolicy(Slots& slots, const Args& args)
{
  slots.template run<Policy>(args);
}

// run specific policy on slots with arguemnts for non-static policy class
template<typename Policy, typename Args, typename Slots>
inline void runSlots(Slots& slots, const Policy& policy, const Args& args)
{
  slots.template run(policy, args);
}


template<typename T1>
inline Slots<T1> tiePlugins(const T1& a1)
{
  return Slots<T1>(a1);
}

template<typename T1, typename T2>
inline Slots<T1,T2> tiePlugins(const T1& a1, const T2& a2)
{
  return Slots<T1,T2>(a1,a2);
}

template<typename T1, typename T2, typename T3>
inline Slots<T1,T2,T3> tiePlugins(const T1& a1, const T2& a2, const T3& a3)
{
  return Slots<T1,T2,T3>(a1,a2,a3);
}

template<typename T1, typename T2, typename T3, typename T4>
inline Slots<T1,T2,T3,T4> tiePlugins(const T1& a1, const T2& a2,
                                     const T3& a3, const T4& a4)
{
  return Slots<T1,T2,T3,T4>(a1,a2,a3,a4);
}

template<typename T1, typename T2, typename T3, typename T4, typename T5>
inline Slots<T1,T2,T3,T4,T5> tiePlugins(const T1& a1, const T2& a2, const T3& a3,
                                     const T4& a4, const T5& a5)
{
  return Slots<T1,T2,T3,T4,T5>(a1,a2,a3,a4,a5);
}

#endif
