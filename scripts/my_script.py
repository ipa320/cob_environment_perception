#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
roslib.load_manifest('cob_sdh')
roslib.load_manifest('cob_torso')
roslib.load_manifest('cob_tray')
roslib.load_manifest('cob_arm')
import rospy

from simple_script_server import script

class MyScript(script):
    def Initialize(self):
        rospy.loginfo("Initializing all components...")

       # self.sss.init("tray")
       # self.sss.init("torso")
       # self.sss.init("arm")
       # self.sss.init("sdh")
        
	   #self.sss.move("arm","wavein")
       # self.sss.move("torso","home")
       # self.sss.move("tray","down")
       # self.sss.move("sdh","home")
       # self.sss.move("base","home")
	


    def Run(self):
        rospy.loginfo("Running script...")
     	self.sss.move("arm","overtray")
        self.sss.move("torso","back")
       # self.sss.move("torso","home")
        self.sss.move("base",[-2.2, -1.7, 0])
       # self.sss.move("torso","back")
       # self.sss.move("base",[-2, -1, 0])
      # self.sss.move("base",[-2, 0, 0])
	  #  self.sss.move("torso","home")
   #     self.sss.move("base",[-2, 0, 0])
       # self.sss.move("base",[-2, 0, 0])
        self.sss.move("base",[-2.2, 1.7, 0])
        self.sss.move("base",[0, 0, 0])
        

        

if __name__ == "__main__":
    SCRIPT = MyScript()
    SCRIPT.Start()
