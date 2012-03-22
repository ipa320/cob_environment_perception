#include "ros/ros.h"
#include "std_msgs/String.h"
#include <brics_actuator/JointPositions.h>		//brics
#include <brics_actuator/JointVelocities.h>		//brics

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "test_client");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<brics_actuator::JointPositions>("/cob_3d_mapping_demonstrator_controller/command_pos", 1);
  //  ros::Publisher chatter_pub = n.advertise<brics_actuator::JointVelocities>("/cob_3d_mapping_demonstrator_controller/command_vel", 1);

  ros::Rate loop_rate(20);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  float count = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    brics_actuator::JointPositions jp;
 //   brics_actuator::JointVelocities jv;
    brics_actuator::JointValue jointVal;
    jointVal.timeStamp = ros::Time::now();
    
    jointVal.joint_uri = "stepper";
    jointVal.unit = "rad";
    printf("Enter stepper target pos (rad): ");
    scanf("%f", &count);
    if(count == 9999)
    	return 0;
    	
    jointVal.value = (double)count;

    jp.positions.push_back(jointVal);

    jointVal.joint_uri = "servo";
    jointVal.unit = "rad";
    printf("Enter servo target pos (rad): ");
    scanf("%f", &count);
    jointVal.value = (double)count;

    jp.positions.push_back(jointVal);

	chatter_pub.publish(jp);


    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
   
  //  chatter_pub.publish(jv);

    ros::spinOnce();

//    loop_rate.sleep();
  }
  return 0;
}
