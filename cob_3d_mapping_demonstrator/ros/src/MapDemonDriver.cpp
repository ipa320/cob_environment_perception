#include <ros/ros.h>
#include <urdf/model.h>
//#include <pthread.h>
#include <actionlib/server/simple_action_server.h>

//ROS Message Includes
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

//ROS Service Includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

//Own includes
#include <cob_3d_mapping_demonstrator/MapDemonCtrlParams.h>
#include <cob_3d_mapping_demonstrator/MapDemonCtrl.h>
#include <cob_3d_mapping_demonstrator/SerialDevice.h>

//using namespace serial_com;

class MappingDemonstratorNode
{
public:
	/// create a handle for this node, initialize node
	ros::NodeHandle n_;

	/// declaration of topics to publish
	ros::Publisher topicPub_JointState_;
	ros::Publisher topicPub_OperationMode_;
	ros::Publisher topicPub_Diagnostic_;

	/// declaration of topics to subscribe
	ros::Subscriber topicSub_CommandPos_;
	ros::Subscriber topicSub_CommandVel_;

	/// declaration of service servers
	ros::ServiceServer srvServer_Init_;
	ros::ServiceServer srvServer_Stop_;
	ros::ServiceServer srvServer_Recover_;
	ros::ServiceServer srvServer_OperationMode_;

	actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> as_;

	/// handle for 3d-mapping-demon ctrl
	MapDemonCtrl* md_ctrl_;

	/// handle for 3d-mapping-demon parameters
	MapDemonCtrlParams* md_params_;

	/// handle for Serial port
	SerialDevice* md_sd_;

	/// member variables
	bool initialized_;
	bool stopped_;
	bool error_;
	bool auto_init;
	std::string error_msg_;
	ros::Time last_publish_time_;

	///Constructor
	MappingDemonstratorNode()
	  :n_("~"),
	   as_(n_, "joint_trajectory_action", boost::bind(&MappingDemonstratorNode::executeTrajectory, this, _1), true)
	{
		//n_ = ros::NodeHandle("~");
		md_sd_ = new SerialDevice();

		md_params_ = new MapDemonCtrlParams();
		md_ctrl_ = new MapDemonCtrl(md_params_, md_sd_);

		/// implementation of topics to publish
		topicPub_JointState_ = n_.advertise<sensor_msgs::JointState> ("joint_states", 1);
		topicPub_OperationMode_ = n_.advertise<std_msgs::String> ("current_operationmode", 1);
		topicPub_Diagnostic_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);

		/// implementation of topics to subscribe
		topicSub_CommandPos_ = n_.subscribe("command_pos", 1, &MappingDemonstratorNode::topicCallback_CommandPos, this);
		topicSub_CommandVel_ = n_.subscribe("command_vel", 1, &MappingDemonstratorNode::topicCallback_CommandVel, this);

		/// implementation of service servers
		srvServer_Init_ = n_.advertiseService("init", &MappingDemonstratorNode::srvCallback_Init, this);
		srvServer_Stop_ = n_.advertiseService("stop", &MappingDemonstratorNode::srvCallback_Stop, this);
		srvServer_Recover_ = n_.advertiseService("recover", &MappingDemonstratorNode::srvCallback_Recover, this);
	    srvServer_OperationMode_ = n_.advertiseService("set_operation_mode", &MappingDemonstratorNode::srvCallback_SetOperationMode, this);

		initialized_ = false;
		stopped_ = true;
		error_ = false;
		last_publish_time_ = ros::Time::now();
	}

	///Destructor
	~MappingDemonstratorNode()
	{

	}

	///Get ROS parameters from parameter server
	void getROSParameters()
	{
		/// get SerialDevice
		std::string SerialDevice;
		SerialDevice = "ttyUSB0";
		if(n_.hasParam("serial_device"))
		{
			n_.getParam("serial_device", SerialDevice);
		}
		ROS_INFO("Serial device set to:\t\t%s", SerialDevice.c_str());

		/// get Baud Rate from parameter server
		int SerialBaudRate;
		SerialBaudRate = 57600;
		if(n_.hasParam("serial_baudrate"))
		{
			n_.getParam("serial_baudrate", SerialBaudRate);
		}
		ROS_INFO("Serial baudrate set to:\t\t%d bps.", SerialBaudRate);
		/// Initialize port parameters and operation mode
		md_params_->Init(SerialDevice, SerialBaudRate);

		/// Set joint names
		XmlRpc::XmlRpcValue JointNamesXmlRpc;
		std::vector<std::string> JointNames;
		if (n_.hasParam("joint_names"))
		{
			n_.getParam("joint_names", JointNamesXmlRpc);
			JointNames.resize(JointNamesXmlRpc.size());
		    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
			{
				JointNames[i] = (std::string)JointNamesXmlRpc[i];
			}
			md_params_->SetJointNames(JointNames);
		}
		else
		{
			ROS_ERROR("Parameter joint_names not set, shutting down node...");
			n_.shutdown();
		}

		/// get operation mode from parameter server
		std::string opmode = "velocity";
		if(n_.hasParam("operation_mode"))
		{
			n_.getParam("operation_mode", opmode);
			md_params_->SetOperationMode(opmode);
		}
		ROS_INFO("Operation mode set to: %s.", opmode.c_str());

		XmlRpc::XmlRpcValue fixedVelocitiesXmlRpc;
		std::vector<double> fixedVelocities;
		if(n_.hasParam("fixed_velocities"))
		{
			n_.getParam("fixed_velocities", fixedVelocitiesXmlRpc);
			fixedVelocities.resize(fixedVelocitiesXmlRpc.size());
			for (int i = 0; i < fixedVelocitiesXmlRpc.size(); i++)
			{
				fixedVelocities[i] = (double)fixedVelocitiesXmlRpc[i];
			}
			md_params_->SetFixedVels(fixedVelocities);
		}
		ROS_INFO("Loaded position fixed_velocities");

			JointNames.resize(JointNamesXmlRpc.size());
		    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
			{
				JointNames[i] = (std::string)JointNamesXmlRpc[i];
			}

		auto_init = true;
		if(n_.hasParam("auto_initialize"))
		{
			n_.getParam("auto_initialize", auto_init);
		}
		ROS_INFO("Auto initialize set to: %d", auto_init);

		ROS_INFO("Parameters initialisation successful.");
	}

	void getRobotDescriptionParameters()
	{
	 	std::vector<std::string> JointNames = md_params_->GetJointNames();
		unsigned int DOF = JointNames.size();	/// always 2 for mapping-demon
		md_params_->SetDOF(DOF);/// set DOF

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
		std::vector<double> MaxVelocities(DOF);
		for (unsigned int i = 0; i < DOF; i++)
		{
			MaxVelocities[i] = model.getJoint(JointNames[i].c_str())->limits->velocity;
		}
		/// Get lower limits out of urdf model
		std::vector<double> LowerLimits(DOF);
		for (unsigned int i = 0; i < DOF; i++)
		{
			LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower;
		}

		// Get upper limits out of urdf model
		std::vector<double> UpperLimits(DOF);
		for (unsigned int i = 0; i < DOF; i++)
		{
			UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper;
		}

		/// Get offsets out of urdf model
		std::vector<double> Offsets(DOF);
		for (unsigned int i = 0; i < DOF; i++)
		{
			Offsets[i] = model.getJoint(JointNames[i].c_str())->calibration->rising.get()[0];
		}

		/// Set parameters
		md_params_->SetMaxVel(MaxVelocities);
		md_params_->SetLowerLimits(LowerLimits);
		md_params_->SetUpperLimits(UpperLimits);
		md_params_->SetOffsets(Offsets);

		ROS_DEBUG("Loading complete");
	}

	void runAutoInit()
	{
		if (md_ctrl_->Init(md_params_))
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

	void executeTrajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal)
	{
	  trajectory_msgs::JointTrajectory traj = goal->trajectory;
	  brics_actuator::JointPositions::Ptr joint_pos(new brics_actuator::JointPositions());
	  for(unsigned int i=0; i<traj.joint_names.size(); i++)
	  {
	    brics_actuator::JointValue jv;
	    jv.joint_uri = traj.joint_names[i];
	    jv.unit = "rad";
	    jv.value = traj.points[0].positions[i];
	    joint_pos->positions.push_back(jv);
	  }
	  topicCallback_CommandPos(joint_pos);
	  bool isMoving = true;
	  while(isMoving)
	  {
	    std::vector<double> pos = md_ctrl_->GetPositions();
	    ROS_INFO("%f, %f", traj.points[0].positions[0]-pos[0], traj.points[0].positions[1]-pos[1]);
	    if( fabs(traj.points[0].positions[0]-pos[0])<0.005 &&  fabs(traj.points[0].positions[1]-pos[1])<0.008 )
	      isMoving = false;
	    if ( as_.isPreemptRequested())
	    {
	      as_.setPreempted();
	      return;
	    }
	    usleep(1000);
	  }
	  as_.setSucceeded();
	}


	void topicCallback_CommandPos(const brics_actuator::JointPositions::ConstPtr& msg)
	{
		ROS_DEBUG("Received new position command.");
		if(initialized_)
		{
			unsigned int DOF = md_params_->GetDOF();
			std::vector<std::string> JointNames = md_params_->GetJointNames();
			std::vector<double> cmd_pos(DOF);
			std::vector<double> lowerLimits = md_params_->GetLowerLimits();
			std::vector<double> upperLimits = md_params_->GetUpperLimits();
			std::vector<double> maxVels = md_params_->GetMaxVel();
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
				if (msg->positions[i].joint_uri != JointNames[i])
				{
					ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.", msg->positions[i].joint_uri.c_str(), JointNames[i].c_str(), i);
					return;
				}

				/// check angular unit
				if (msg->positions[i].unit != unit)
				{
					ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.", msg->positions[i].unit.c_str(), unit.c_str());
					return;
				}

				/// check angular limits
				if(msg->positions[i].value > upperLimits[i])
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
				{
					/// if all checks are successful, parse the position value for this joint
					ROS_DEBUG("Parsing of position %f for joint %s successful.", msg->positions[i].value, JointNames[i].c_str());
					cmd_pos[i] = msg->positions[i].value;
				}
			}
			// send both positions in the vector. '1' is ok. Any negative number is the index of the joint with error. 0 means that Jointnames were incorrect
			if( !md_ctrl_->MovePos(cmd_pos) )
			{
				error_ = true;
				error_msg_ = md_ctrl_->getErrorMessage();
				ROS_ERROR("Joint reposition error: %s", md_ctrl_->getErrorMessage().c_str());
				return;
			}

			ROS_INFO("Successfully executed position command");
		}
		else
		{
			ROS_WARN("Skipping command: mapping_demonstrator is not initialized");
		}
	}

	void topicCallback_CommandVel(const brics_actuator::JointVelocities::ConstPtr& msg)
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
	}

	bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
	{
		if (!initialized_)
		{
			ROS_INFO("Initializing COB3DMD...");

			  /// initialize
			if (md_ctrl_->Init(md_params_))
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
	bool srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
	{
		if( initialized_ )
		{
			if( md_ctrl_->Stop() )
			{
				ROS_INFO("Stopping COB3DMD successful");
				res.success.data = true;	//Trigger.srv response
				stopped_ = true;
			}
			else
			{
				res.success.data = false;
				md_ctrl_->getErrorMessage();
				ROS_ERROR("...stopping COB3DMD unsuccessful. Error: %s", res.error_message.data.c_str());
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

	bool srvCallback_Recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
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
			if( !md_ctrl_->Recover() )
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

	bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res)
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
	}
};	// MapDemonDriverNode

bool publisher( void *arg )
{
	MappingDemonstratorNode * md_node = (MappingDemonstratorNode*) arg;
	md_node->last_publish_time_ = ros::Time::now();

	if(md_node->initialized_)	/// don't publish any of this if not initialised
	{
		if( md_node->md_ctrl_->UpdatePositions() )
		{
			/// create JointState message and publish
			sensor_msgs::JointState joint_state_msg;
			joint_state_msg.header.stamp = ros::Time::now();
			joint_state_msg.header.frame_id = "COB3DMD";
			joint_state_msg.name = md_node->md_params_->GetJointNames();
			joint_state_msg.position = md_node->md_ctrl_->GetPositions();
			joint_state_msg.velocity = md_node->md_ctrl_->GetVelocities();

			ROS_DEBUG("Publishing COB3DMD state");
			md_node->topicPub_JointState_.publish(joint_state_msg);
		}
		else
		{
			ROS_ERROR("Pan reported position incongruency. Run recal");
			md_node->error_ = true;
		}
	}

	/// publish operation mode topic
	std_msgs::String opmode_msg;
	opmode_msg.data = md_node->md_params_->GetOperationMode();
	md_node->topicPub_OperationMode_.publish(opmode_msg);

	// publishing diagnotic messages
	diagnostic_msgs::DiagnosticArray diagnostics;
	diagnostics.status.resize(1);

	// set diagnostics
	if(md_node->error_)
	{
		diagnostics.status[0].level = 2;
		diagnostics.status[0].name = md_node->n_.getNamespace();;
		diagnostics.status[0].message = md_node->md_ctrl_->getErrorMessage();
	}
	else
	{
		if (md_node->initialized_)
		{
			diagnostics.status[0].level = 0;
			diagnostics.status[0].name = md_node->n_.getNamespace();
			diagnostics.status[0].message = "cob_3d_mapping_demonstrator is running";
		}
		else
		{
			diagnostics.status[0].level = 1;
			diagnostics.status[0].name = md_node->n_.getNamespace();
			diagnostics.status[0].message = "cob_3d_mapping_demonstrator not initialized";
		}
	}
	// publish diagnostic message
	md_node->topicPub_Diagnostic_.publish(diagnostics);

	return true;
}

int main(int argc, char **argv)
{
	//pthread_t th;   	//publisher thread

	/// initialize ROS, specify name of node
	ros::init(argc, argv, "cob_3d_mapping_demonstrator_node");

	MappingDemonstratorNode md_node;	/// create node, already initializing

	md_node.getROSParameters();					/// get configuration parameters from parameter server
	md_node.getRobotDescriptionParameters();	/// get robot parameters from URDF file
	if( md_node.auto_init )
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
		frequency = 20 ;	//Hz
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
			publisher(&md_node);
		}

		/// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
