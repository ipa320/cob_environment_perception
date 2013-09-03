#include <ros/ros.h>
#include <urdf/model.h>
//#include <pthread.h>
#include <actionlib/server/simple_action_server.h>

//ROS Message Includes
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

//ROS Service Includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

//Own includes
#include <cob_3d_mapping_demonstrator/MapDemonCtrlParams.h>
#include <cob_3d_mapping_demonstrator/MapDemonCtrl.h>
#include <cob_3d_mapping_demonstrator/MapDemonCtrl_Maestro.h>
//#include <cob_3d_mapping_demonstrator/SerialDevice.h>

//using namespace serial_com;

class DemonstratorNode
{
public:
  /// create a handle for this node, initialize node
  ros::NodeHandle n_;

  /// declaration of topics to publish
  ros::Publisher topic_pub_joint_state_;
  //ros::Publisher topic_pub_operation_mode_;
  ros::Publisher topic_pub_diagnostic_;

  /// declaration of topics to subscribe
  ros::Subscriber topic_sub_command_pos_;
  //ros::Subscriber topic_sub_command_vel_;

  /// declaration of service servers
  ros::ServiceServer srv_server_init_;
  ros::ServiceServer srv_server_stop_;
  ros::ServiceServer srv_server_recover_;
  //ros::ServiceServer srv_server_operation_mode_;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

  /// handle for 3d-mapping-demon ctrl
  MapDemonCtrlMaestro* md_ctrl_;

  /// handle for 3d-mapping-demon parameters
  MapDemonCtrlParams* md_params_;

  /// handle for Serial port
  //SerialDevice* md_sd_;

  /// member variables
  bool initialized_;
  bool stopped_;
  bool error_;
  bool auto_init_;
  std::string error_msg_;
  ros::Time last_publish_time_;

  ///Constructor
  DemonstratorNode()
  :n_("~"),
   as_(n_, "follow_joint_trajectory", boost::bind(&DemonstratorNode::executeTrajectory, this, _1), false)
  {
    //n_ = ros::NodeHandle("~");
    //md_sd_ = new SerialDevice();

    md_params_ = new MapDemonCtrlParams();
    md_ctrl_ = new MapDemonCtrlMaestro(md_params_);

    /// implementation of topics to publish
    topic_pub_joint_state_ = n_.advertise<sensor_msgs::JointState> ("joint_states", 1);
    //topic_pub_operation_mode_ = n_.advertise<std_msgs::String> ("current_operationmode", 1);
    topic_pub_diagnostic_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);

    /// implementation of topics to subscribe
    topic_sub_command_pos_ = n_.subscribe("command_pos", 1, &DemonstratorNode::topicCallbackCommandPos, this);
    //topic_sub_command_vel_ = n_.subscribe("command_vel", 1, &DemonstratorNode::topicCallbackCommandVel, this);

    /// implementation of service servers
    srv_server_init_ = n_.advertiseService("init", &DemonstratorNode::srvCallbackInit, this);
    srv_server_stop_ = n_.advertiseService("stop", &DemonstratorNode::srvCallbackStop, this);
    srv_server_recover_ = n_.advertiseService("recover", &DemonstratorNode::srvCallbackRecover, this);
    //srv_server_operation_mode_ = n_.advertiseService("set_operation_mode", &DemonstratorNode::srvCallbackSetOperationMode, this);

    as_.start();

    initialized_ = false;
    stopped_ = true;
    error_ = false;
    last_publish_time_ = ros::Time::now();
  }

  ///Destructor
  ~DemonstratorNode()
  {
    delete md_ctrl_;
  }

  ///Get ROS parameters from parameter server
  bool getROSParameters()
  {
    /// get serial_device
    std::string serial_device = "ttyUSB0";
    if(n_.hasParam("serial_device"))
    {
      n_.getParam("serial_device", serial_device);
    }
    ROS_INFO("Serial device set to:\t\t%s", serial_device.c_str());

    /// get Baud Rate from parameter server
    int serial_baud_rate = 57600;
    if(n_.hasParam("serial_baudrate"))
    {
      n_.getParam("serial_baudrate", serial_baud_rate);
    }
    ROS_INFO("Serial baudrate set to:\t\t%d bps.", serial_baud_rate);
    /// Initialize port parameters and operation mode
    md_params_->init(serial_device, serial_baud_rate);

    /// Set joint names
    XmlRpc::XmlRpcValue joint_names_xml_rpc;
    std::vector<std::string> joint_names;
    if (n_.hasParam("joint_names"))
    {
      n_.getParam("joint_names", joint_names_xml_rpc);
      joint_names.resize(joint_names_xml_rpc.size());
      for (int i = 0; i < joint_names_xml_rpc.size(); i++)
      {
        joint_names[i] = (std::string)joint_names_xml_rpc[i];
      }
      md_params_->setJointNames(joint_names);
    }
    else
    {
      ROS_ERROR("Parameter joint_names not set, shutting down node...");
      return false;
    }

    /// get operation mode from parameter server
    //std::string opmode = "position";
    //if(n_.hasParam("operation_mode"))
    //{
    //  n_.getParam("operation_mode", opmode);
    //  md_params_->setOperationMode(opmode);
    //}
    //ROS_INFO("Operation mode set to: %s.", opmode.c_str());

    XmlRpc::XmlRpcValue velocities_xml_rpc;
    std::vector<double> velocities;
    if(n_.hasParam("velocities"))
    {
      n_.getParam("velocities", velocities_xml_rpc);
      velocities.resize(velocities_xml_rpc.size());
      for (int i = 0; i < velocities_xml_rpc.size(); i++)
      {
        velocities[i] = (double)velocities_xml_rpc[i];
      }
      md_params_->setVels(velocities);
    }
    ROS_INFO("Loaded position velocities");

    XmlRpc::XmlRpcValue accels_xml_rpc;
    std::vector<double> accelerations;
    if(n_.hasParam("accelerations"))
    {
      n_.getParam("accelerations", accels_xml_rpc);
      accelerations.resize(accels_xml_rpc.size());
      for (int i = 0; i < accels_xml_rpc.size(); i++)
      {
        accelerations[i] = (double)accels_xml_rpc[i];
      }
      md_params_->setAccels(accelerations);
    }
    ROS_INFO("Loaded position accelerations");

    joint_names.resize(joint_names_xml_rpc.size());
    for (int i = 0; i < joint_names_xml_rpc.size(); i++)
    {
      joint_names[i] = (std::string)joint_names_xml_rpc[i];
    }

    auto_init_ = true;
    if(n_.hasParam("auto_initialize"))
    {
      n_.getParam("auto_initialize", auto_init_);
    }
    ROS_INFO("Auto initialize set to: %d", auto_init_);

    ROS_INFO("Parameters initialisation successful.");
  }

  void getRobotDescriptionParameters()
  {
    std::vector<std::string> joint_names = md_params_->getJointNames();
    unsigned int dof = joint_names.size();	/// always 2 for mapping-demon
    md_params_->setDOF(dof);/// set DOF

    urdf::Model model;
    ROS_DEBUG("Loading urdf");
    //Get robot urdf xml string from parameter server
    std::string param_descr = "/robot_description";
    if (n_.hasParam(param_descr))
    {
      model.initParam(param_descr);
      ROS_INFO("Successfuly loaded URDF description.");
    }
    else
    {
      ROS_ERROR("URDF not found, shutting down node...");
      n_.shutdown();
    }

    ///This tries to extract the parameters from the urdf file
    /// Get max velocities out of urdf model
    std::vector<double> max_velocities(dof);
    for (unsigned int i = 0; i < dof; i++)
    {
      max_velocities[i] = model.getJoint(joint_names[i].c_str())->limits->velocity;
    }
    /// Get lower limits out of urdf model
    std::vector<double> lower_limits(dof);
    for (unsigned int i = 0; i < dof; i++)
    {
      lower_limits[i] = model.getJoint(joint_names[i].c_str())->limits->lower;
    }

    // Get upper limits out of urdf model
    std::vector<double> upper_limits(dof);
    for (unsigned int i = 0; i < dof; i++)
    {
      upper_limits[i] = model.getJoint(joint_names[i].c_str())->limits->upper;
    }

    /// Get offsets out of urdf model
    std::vector<double> offsets(dof);
    for (unsigned int i = 0; i < dof; i++)
    {
      offsets[i] = model.getJoint(joint_names[i].c_str())->calibration->rising.get()[0];
    }

    /// Set parameters
    md_params_->setMaxVel(max_velocities);
    md_params_->setLowerLimits(lower_limits);
    md_params_->setUpperLimits(upper_limits);
    md_params_->setOffsets(offsets);

    ROS_DEBUG("Loading complete");
  }

  void runAutoInit()
  {
    if (md_ctrl_->init(md_params_))
    {
      initialized_ = true;
      ROS_INFO("...initializing COB3DMD successful");
    }
    else
    {
      error_ = true;
      error_msg_ = md_ctrl_->getErrorMessage();
      ROS_ERROR("...initializing COB3DMD unsuccessful. Error: %s", error_msg_.c_str());
    }

  }

  void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    //ROS_INFO("trajectory");
    stopped_ = false;
    trajectory_msgs::JointTrajectory traj = goal->trajectory;
    for( unsigned int j = 0; j <traj.points.size(); j++)
    {
      brics_actuator::JointPositions::Ptr joint_pos(new brics_actuator::JointPositions());
      for(unsigned int i=0; i<traj.joint_names.size(); i++)
      {
        brics_actuator::JointValue jv;
        jv.joint_uri = traj.joint_names[i];
        jv.unit = "rad";
        jv.value = traj.points[j].positions[i];
        joint_pos->positions.push_back(jv);
      }
      topicCallbackCommandPos(joint_pos);
      while(!md_ctrl_->is_moving_ && !stopped_) usleep(1000);
      while(md_ctrl_->is_moving_)
      {
        //is_moving_ = md_ctrl_->is_moving_;
        usleep(10000);
      }
    /*while(is_moving)
    {
      std::vector<double> pos = md_ctrl_->getPositions();
      //ROS_INFO("%f, %f", traj.points[0].positions[0], pos[0]);
      if( fabs(traj.points[0].positions[0]-pos[0])<0.05 &&  fabs(traj.points[0].positions[1]-pos[1])<0.05 )
        is_moving = false;
      if ( as_.isPreemptRequested() || stopped_)
      {
        as_.setPreempted();
        return;
      }
      usleep(1000);
    }*/
    }
    as_.setSucceeded();
  }


  void topicCallbackCommandPos(const brics_actuator::JointPositions::ConstPtr& msg)
  {
    ROS_DEBUG("Received new position command.");
    if(!initialized_)
    {
      ROS_WARN("Skipping command: mapping_demonstrator is not initialized");
      return;
    }
    if(stopped_) return;

    unsigned int DOF = md_params_->getDOF();
    std::vector<std::string> joint_names = md_params_->getJointNames();
    std::vector<double> cmd_pos(DOF);
    /*std::vector<double> lowerLimits = md_params_->getLowerLimits();
    std::vector<double> upperLimits = md_params_->getUpperLimits();
    std::vector<double> maxVels = md_params_->getMaxVel();*/
    std::string unit = "rad";

    /// check dimensions
    if (msg->positions.size() != DOF)
    {
      ROS_ERROR("Skipping command: Commanded positions and DOF are not same dimension.");
      return;
    }

    /// parse positions
    for (unsigned int i = 0; i < DOF; i++)
    {
      /// check joint name
      if (msg->positions[i].joint_uri != joint_names[i])
      {
        ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.", msg->positions[i].joint_uri.c_str(), joint_names[i].c_str(), i);
        return;
      }

      /// check angular unit
      if (msg->positions[i].unit != unit)
      {
        ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.", msg->positions[i].unit.c_str(), unit.c_str());
        return;
      }

      /// check angular limits
      /*if(msg->positions[i].value > upperLimits[i])
      {
        ROS_WARN("Position %f exceeds limit %f for axis %s.", msg->positions[i].value, upperLimits[i], JointNames[i].c_str());
        cmd_pos[i] = upperLimits[i];	// command upperLimit position
      }
      else if(msg->positions[i].value < lowerLimits[i])
      {
        ROS_WARN("Position %f exceeds limit %f for axis %s.", msg->positions[i].value, lowerLimits[i], JointNames[i].c_str());
        cmd_pos[i] = lowerLimits[i];	// command lowerLimit position
      }
      else
      {*/
        /// if all checks are successful, parse the position value for this joint
      ROS_DEBUG("Parsing of position %f for joint %s successful.", msg->positions[i].value, joint_names[i].c_str());
      cmd_pos[i] = msg->positions[i].value;
      //}
    }
    // send both positions in the vector. '1' is ok. Any negative number is the index of the joint with error. 0 means that Jointnames were incorrect
    if( !md_ctrl_->movePos(cmd_pos) )
    {
      error_ = true;
      error_msg_ = md_ctrl_->getErrorMessage();
      ROS_ERROR("Joint reposition error: %s", md_ctrl_->getErrorMessage().c_str());
      return;
    }

    ROS_INFO("Successfully executed position command");
  }

  /*void topicCallbackCommandVel(const brics_actuator::JointVelocities::ConstPtr& msg)
  {
    ROS_DEBUG("Received new velocity command.");

    if(initialized_)
    {
      unsigned int DOF = md_params_->GetDOF();
      std::vector<std::string> JointNames = md_params_->GetJointNames();
      std::vector<double> cmd_vel(DOF);
      std::vector<double> maxVels = md_params_->GetMaxVel();
      std::string unit = "rad";

      /// check dimensions
      if (msg->velocities.size() != DOF)
      {
        ROS_ERROR("Skipping command: Commanded velocities and DOF are not same dimension.");
        return;
      }

      /// parse positions
      for (unsigned int i = 0; i < DOF; i++)
      {
        /// check joint name
        if (msg->velocities[i].joint_uri != JointNames[i])
        {
          ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.", msg->velocities[i].joint_uri.c_str(), JointNames[i].c_str(), i);
          return;
        }

        /// check angular unit
        if (msg->velocities[i].unit != unit)
        {
          ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.", msg->velocities[i].unit.c_str(), unit.c_str());
          return;
        }

        /// check angular limits
        if(fabs(msg->velocities[i].value) > maxVels[i])
        {
          ROS_WARN("Velocity %f exceeds limit %f for axis %s.", msg->velocities[i].value, maxVels[i], JointNames[i].c_str());
          cmd_vel[i] = maxVels[i];	// command upperLimit position
        }
        else
        {
          /// if all checks are successful, parse the velocity value for this joint
          ROS_DEBUG("Parsing of velocities %f for joint %s successful.", msg->velocities[i].value, JointNames[i].c_str());
          cmd_vel[i] = msg->velocities[i].value;
        }
      }

      if (!md_ctrl_->MoveVel(cmd_vel))	// send both positions in the vector
      {
        error_ = true;
        error_msg_ = md_ctrl_->getErrorMessage();
        ROS_ERROR("Skipping command: %s", md_ctrl_->getErrorMessage().c_str());
        return;
      }

      ROS_DEBUG("Executed velocity command");
    }
    else
    {
      ROS_WARN("Skipping command: mapping_demonstrator is not initialized");
    }
  }*/

  bool srvCallbackInit(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
    if (!initialized_)
    {
      ROS_INFO("Initializing COB3DMD...");

      /// initialize
      if (md_ctrl_->init(md_params_))
      {
        initialized_ = true;
        res.success.data = true;	//Trigger.srv response
        ROS_INFO("...initializing COB3DMD successful");
      }
      else
      {
        error_ = true;
        error_msg_ = md_ctrl_->getErrorMessage();
        res.success.data = false;
        res.error_message.data = md_ctrl_->getErrorMessage();
        ROS_ERROR("...initializing COB3DMD unsuccessful. Error: %s", res.error_message.data.c_str());
      }
    }
    else
    {
      error_ = false;
      res.success.data = false;
      res.error_message.data = "COB3DMD already initialized";
      ROS_WARN("...initializing COB3DMD unsuccessful. Warn: %s", res.error_message.data.c_str());
    }

    return true;
  }
  /*!
   * \brief Stops robot immediately
   */
  bool srvCallbackStop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
    if( initialized_ )
    {
      stopped_ = true;
      if( md_ctrl_->stop() )
      {
        ROS_INFO("Stopping COB3DMD successful");
        res.success.data = true;	//Trigger.srv response
        //stopped_ = true;
      }
      else
      {
        res.success.data = false;
        md_ctrl_->getErrorMessage();
        ROS_ERROR("!!...stopping COB3DMD unsuccessful. Error: %s", res.error_message.data.c_str());
      }
    }
    else
    {
      res.success.data = false;
      res.error_message.data = "Not initialized";
      ROS_WARN("...stopping COB3DMD unsuccessful. Warn: %s", res.error_message.data.c_str());
    }
    return true;
  }

  bool srvCallbackRecover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
  {
    ROS_WARN("Attempting to recover...");

    if (!initialized_)
    {
      error_msg_ = "Not Initialized";
      res.success.data = false;
      res.error_message.data = error_msg_;
      ROS_ERROR("...recovering COB3DMD unsuccessful. Error: %s", error_msg_.c_str());
      return false;
    }
    else
    {
      ROS_WARN("Recalibrating COB3DMD now...");
      if( !md_ctrl_->recover() )
      {
        error_ = true;
        error_msg_ = md_ctrl_->getErrorMessage();
        res.success.data = false;
        res.error_message.data = md_ctrl_->getErrorMessage();
        ROS_ERROR("...recovering COB3DMD unsuccessful. Error: %s", res.error_message.data.c_str());
        return false;
      }
      else
      {
        error_ = false;
        error_msg_ = "";
        res.success.data = true;
        res.error_message.data = error_msg_;
        ROS_INFO("...recovering COB3DMD successful");
        return true;
      }
    }
  }

  /*bool srvCallbackSetOperationMode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res)
  {
    if (req.operation_mode.data == "velocity")
    {
      md_params_->SetOperationMode("velocity");
      res.success.data = true;
    }
    else if (req.operation_mode.data == "position")
    {
      md_params_->SetOperationMode("position");
      res.success.data = true;
    }
    else
    {
      res.success.data = false;
    }

    ROS_INFO("Operation mode change request to %s succeded.", req.operation_mode.data.c_str());
    return true;
  }*/

  bool publishStates()
  {
    last_publish_time_ = ros::Time::now();

    if(initialized_)     /// don't publish any of this if not initialised
    {
      if( md_ctrl_->updatePositions() )
      {
        /// create JointState message and publish
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.header.frame_id = "COB3DMD";
        joint_state_msg.name = md_params_->getJointNames();
        joint_state_msg.position = md_ctrl_->getPositions();
        joint_state_msg.velocity = md_ctrl_->getVelocities();

        ROS_DEBUG("Publishing COB3DMD state");
        topic_pub_joint_state_.publish(joint_state_msg);
      }
      else
      {
        ROS_ERROR("Pan reported position incongruency. Run recal");
        error_ = true;
      }
    }

    /// publish operation mode topic
    /*std_msgs::String opmode_msg;
    opmode_msg.data = md_node->md_params_->GetOperationMode();
    md_node->topic_pub_operation_mode_.publish(opmode_msg);*/

    // publishing diagnotic messages
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.status.resize(1);

    // set diagnostics
    if(error_)
    {
      diagnostics.status[0].level = 2;
      diagnostics.status[0].name = n_.getNamespace();;
      diagnostics.status[0].message = md_ctrl_->getErrorMessage();
    }
    else
    {
      if (initialized_)
      {
        diagnostics.status[0].level = 0;
        diagnostics.status[0].name = n_.getNamespace();
        diagnostics.status[0].message = "cob_3d_mapping_demonstrator is running";
      }
      else
      {
        diagnostics.status[0].level = 1;
        diagnostics.status[0].name = n_.getNamespace();
        diagnostics.status[0].message = "cob_3d_mapping_demonstrator not initialized";
      }
    }
    // publish diagnostic message
    topic_pub_diagnostic_.publish(diagnostics);

    return true;
  }
};	// MapDemonDriverNode


int main(int argc, char **argv)
{
  //pthread_t th;   	//publisher thread

  /// initialize ROS, specify name of node
  ros::init(argc, argv, "cob_3d_mapping_demonstrator_node");

  DemonstratorNode md_node;	/// create node, already initializing

  if(!md_node.getROSParameters()) return 0;					/// get configuration parameters from parameter server
  md_node.getRobotDescriptionParameters();	/// get robot parameters from URDF file
  if( md_node.auto_init_ )
    md_node.runAutoInit();

  /// set node loop rate in Hz
  double frequency;
  if (md_node.n_.hasParam("frequency"))
  {
    md_node.n_.getParam("frequency", frequency);
    ROS_INFO("Parameter frequency set to %f Hz", frequency);
  }
  else
  {
    frequency = 50 ;	//Hz
    ROS_WARN("Parameter frequency not defined, setting to %f Hz", frequency);
  }

  /// set node publisher latency in seconds
  ros::Duration min_publish_duration;
  if (md_node.n_.hasParam("min_publish_duration"))
  {
    double sec;
    md_node.n_.getParam("min_publish_duration", sec);
    ROS_INFO("Parameter min publish duration set to %f seconds", sec);
    min_publish_duration.fromSec(sec);
  }
  else
  {
    min_publish_duration.fromSec(1 / frequency);
    ROS_WARN("Parameter min_publish_duration not defined, setting to %f sec", min_publish_duration.toSec());
  }

  /// main loop
  ros::Rate loop_rate(frequency); // Hz

  while (md_node.n_.ok())
  {
    //if ((ros::Time::now() - md_node.last_publish_time_) >= min_publish_duration)
    {
      md_node.publishStates();
    }

    /// sleep and waiting for messages, callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
