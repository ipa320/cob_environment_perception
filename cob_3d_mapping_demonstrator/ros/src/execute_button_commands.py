#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_environment_perception_intern
# \note
#   ROS package name: cob_3d_mapping_demonstrator
#
# \author
#   Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
# \author
#   Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
#
# \date Date of creation: 03/2012
#
# \brief
#   Implementation of ROS node for script_server.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import time

import roslib
roslib.load_manifest('cob_3d_mapping_demonstrator')
import rospy
import actionlib

from cob_script_server.msg import *
from simple_script_server import *
from cob_srvs.srv import *
from cob_3d_mapping_msgs.msg import *

sss = simple_script_server()

## Script server class which inherits from script class.
#
# Implements actionlib interface for the script server.
#
class execute_button_commands():
  ## Initializes the actionlib interface of the script server.
  #
  def __init__(self):
    self.ns_global_prefix = "/cob_3d_mapping_demonstrator"
    self.script_action_server = actionlib.SimpleActionServer("cob_3d_mapping_demonstrator", ScriptAction, self.execute_cb, False)
    self.script_action_server.register_preempt_callback(self.execute_stop)
    self.script_action_server.start()
    self.trigger_client = actionlib.SimpleActionClient('trigger_mapping', TriggerAction)
    self.step = 0.1

#------------------- Actionlib section -------------------#
  ## Executes actionlib callbacks.
  #
  # \param server_goal ScriptActionGoal
  #
  def execute_cb(self, server_goal):
    print "execute_cb"
    server_result = ScriptActionResult().result
    if server_goal.function_name == "start":
      print "start"
      ret=self.execute_start()
    #elif server_goal.function_name == "stop":
    #  ret=self.execute_stop()
    elif server_goal.function_name == "reset":
      ret=self.execute_reset()
    elif server_goal.function_name == "clear":
      ret=self.execute_clear()
    elif server_goal.function_name == "step":
      ret=self.execute_step()
    elif server_goal.function_name == "recover":
      ret=self.execute_recover()
    else:
      rospy.logerr("function <<%s>> not supported", server_goal.function_name)
      self.script_action_server.set_aborted(server_result)
      return

    #server_result.error_code = handle01.get_error_code()
    #if server_result.error_code == 0:
    #  rospy.logdebug("action result success")
    if self.script_action_server.is_active():
      self.script_action_server.set_succeeded(server_result)
    #else:
    #  self.script_action_server.set_aborted(server_result)
    #else:
    #  rospy.logerr("action result error")
    #  self.script_action_server.set_aborted(server_result)

  def execute_start(self):
    print "start"
    #self.execute_reset()
    #sss.move("cob_3d_mapping_demonstrator",[[-0.5,0]])
        #return False
    #if self.script_action_server.is_preempt_requested():
    #  self.script_action_server.set_preempted()
    #  return False
    print self.script_action_server.is_preempt_requested()
    while not self.script_action_server.is_preempt_requested():
      print "move"
      sss.move("cob_3d_mapping_demonstrator",[[0.5,0],[-0.5,0],[-0.5,-0.5],[0.5,-0.5],[0.5,0],[0,0]])
      sss.sleep(1.0)
    #sss.move("cob_3d_mapping_demonstrator","home")
    #if self.script_action_server.is_preempt_requested():
    #  self.script_action_server.set_preempted()
    #  sss.stop("cob_3d_mapping_demonstrator")
    #  return False
    #sss.move("cob_3d_mapping_demonstrator",[[0.5,0]])
    #sss.sleep(0.5)
    #if self.script_action_server.is_preempt_requested():
    #  self.script_action_server.set_preempted()
    #  sss.stop("cob_3d_mapping_demonstrator")
    #  return False
    #sss.move("cob_3d_mapping_demonstrator",[[0.5,-0.5]])
    #if self.script_action_server.is_preempt_requested():
    #  self.script_action_server.set_preempted()
    #  sss.stop("cob_3d_mapping_demonstrator")
    #  return False
    #sss.move("cob_3d_mapping_demonstrator",[[0,-0.5]])
    #if self.script_action_server.is_preempt_requested():
    #  self.script_action_server.set_preempted()
    #  sss.stop("cob_3d_mapping_demonstrator")
    #  return False
    #sss.move("cob_3d_mapping_demonstrator",[[-0.5,-0.5]])
    #if self.script_action_server.is_preempt_requested():
    #  self.script_action_server.set_preempted()
    #  sss.stop("cob_3d_mapping_demonstrator")
    #  return False
    #sss.move("cob_3d_mapping_demonstrator","home")

  def execute_start_map(self):
    print "start"
    goal = TriggerGoal()
    if not self.trigger_client.wait_for_server(rospy.Duration.from_sec(2.0)):
      print "switched to /segmentation/trigger"
      self.trigger_client = actionlib.SimpleActionClient('/segmentation/trigger', TriggerAction)

    if not self.trigger_client.wait_for_server(rospy.Duration.from_sec(2.0)):
      rospy.logerr('server not available')
      #return False
    else:
      goal.start = True
      self.trigger_client.send_goal(goal)
      if not self.trigger_client.wait_for_result(rospy.Duration.from_sec(2.0)):
        print "no result"
        #return False
    sss.move("cob_3d_mapping_demonstrator",[[-0.5,0]])
    if self.script_action_server.is_preempt_requested():
      self.script_action_server.set_preempted()
      return False
    sss.move("cob_3d_mapping_demonstrator","home")
    if self.script_action_server.is_preempt_requested():
      self.script_action_server.set_preempted()
      sss.stop("cob_3d_mapping_demonstrator")
      return False
    sss.move("cob_3d_mapping_demonstrator",[[0.5,0]])
    sss.sleep(0.5)
    if self.script_action_server.is_preempt_requested():
      self.script_action_server.set_preempted()
      sss.stop("cob_3d_mapping_demonstrator")
      return False
    sss.move("cob_3d_mapping_demonstrator",[[0.5,-0.5]])
    if self.script_action_server.is_preempt_requested():
      self.script_action_server.set_preempted()
      sss.stop("cob_3d_mapping_demonstrator")
      return False
    sss.move("cob_3d_mapping_demonstrator",[[0,-0.5]])
    if self.script_action_server.is_preempt_requested():
      self.script_action_server.set_preempted()
      sss.stop("cob_3d_mapping_demonstrator")
      return False
    sss.move("cob_3d_mapping_demonstrator",[[-0.5,-0.5]])
    if self.script_action_server.is_preempt_requested():
      self.script_action_server.set_preempted()
      sss.stop("cob_3d_mapping_demonstrator")
      return False
    sss.move("cob_3d_mapping_demonstrator","home")
    goal.start = False
    self.trigger_client.send_goal(goal)
    if not self.trigger_client.wait_for_result(rospy.Duration.from_sec(2.0)):
      return False
    return True


  def execute_stop(self):
    print "stop"
    self.script_action_server.set_preempted()
    sss.stop("cob_3d_mapping_demonstrator")
    return True

  def execute_reset(self):
    print "reset"
    self.execute_stop()
    goal = TriggerGoal()
    goal.start = False
    self.trigger_client.send_goal(goal)
    if not self.trigger_client.wait_for_result(rospy.Duration.from_sec(2.0)):
      print "mapping not running"
      return False
    #sss.move("cob_3d_mapping_demonstrator","home")
    #self.execute_clear()
    #self.step = 0.1
    return True

    #TODO: move to home, stop mapping, clear map

  def execute_clear(self):
    print "clear"
    try:
      #rospy.wait_for_service('point_map/clear_point_map', timeout=2.0)
      clear = rospy.ServiceProxy('point_map/clear_map', Trigger)
      resp = clear()
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    try:
      #rospy.wait_for_service('geometry_map/clear_geometry_map', timeout=2.0)
      clear = rospy.ServiceProxy('geometry_map/clear_map', Trigger)
      resp = clear()
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    try:
      #rospy.wait_for_service('geometry_map/clear_geometry_map', timeout=2.0)
      clear = rospy.ServiceProxy('registration/reset', Trigger)
      resp = clear()
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    return True

  def execute_recover(self):
    goal = TriggerGoal()
    if not self.trigger_client.wait_for_server(rospy.Duration.from_sec(2.0)):
      print "switched to /segmentation/trigger"
      self.trigger_client = actionlib.SimpleActionClient('/segmentation/trigger', TriggerAction)

    if not self.trigger_client.wait_for_server(rospy.Duration.from_sec(2.0)):
      rospy.logerr('server not available')
      #return False
    else:
      goal.start = True
      self.trigger_client.send_goal(goal)
      if not self.trigger_client.wait_for_result(rospy.Duration.from_sec(2.0)):
        print "no result"
    #print "recover"
    #sss.recover("cob_3d_mapping_demonstrator")
    return True

  def execute_step(self):
    sss.move("cob_3d_mapping_demonstrator",[[0, -0.5]])
    """if self.step==0.1:
      self.execute_clear()
    print "step"
    goal = TriggerGoal()
    if not self.trigger_client.wait_for_server(rospy.Duration.from_sec(2.0)):
      rospy.logerr('server not available')
      #return False
    else:
      goal.start = True
      self.trigger_client.send_goal(goal)
      if not self.trigger_client.wait_for_result(rospy.Duration.from_sec(2.0)):
        print "no result"
    sss.move("cob_3d_mapping_demonstrator",[[self.step,-0.3]])
    self.step += 0.1"""

## Main routine for running the script server
#
if __name__ == '__main__':
  rospy.init_node('execute_button_commands')
  execute_button_commands()
  rospy.loginfo("execute button commands is running")
  rospy.spin()
