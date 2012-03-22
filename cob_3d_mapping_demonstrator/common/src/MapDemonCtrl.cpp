// ROS includes
#include <ros/ros.h>

// standard includes
#include <boost/thread.hpp>
#include <string>
#include <cstring>
#include <stdlib.h>
#include <math.h>

// own includes
#include <cob_3d_mapping_demonstrator/MapDemonCtrl.h>

MapDemonCtrl::MapDemonCtrl(MapDemonCtrlParams * params, SerialDevice * sd)
{
	m_SerialDeviceOpened = false;
	m_Initialized = false;
	m_last_time_pub = ros::Time::now();
	m_params_ = params;
	m_sd = sd;
}

/// Destructor
MapDemonCtrl::~MapDemonCtrl()
{

}

/*!
 * \brief Initializing
 */
bool MapDemonCtrl::Init(MapDemonCtrlParams * params)
{
	/// get serial port configurable parameters
	std::string SerialDeviceName = m_params_->GetSerialDevice();
	int SerialBaudrate = m_params_->GetBaudRate();
	std::ostringstream errorMsg;	

	int DOF = m_params_->GetDOF();
	std::vector<std::string> JointNames = m_params_->GetJointNames();
	std::vector<double> MaxVel = m_params_->GetMaxVel();
	std::vector<double> FixedVel = m_params_->GetFixedVels();
	std::vector<double> Offsets = m_params_->GetOffsets();
	std::vector<double> LowerLimits = m_params_->GetLowerLimits();
	std::vector<double> UpperLimits = m_params_->GetUpperLimits();
	
	
	m_positions.resize(DOF);
	m_velocities.resize(DOF);
	
	std::cout << "============================================================================== " << std::endl;
	std::cout << "Mapping Demonstrator Init: Trying to initialize with the following parameters: " << std::endl;
	
	std::cout << std::endl << "Joint Names:\t\t";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << JointNames[i] << "\t";
	}
	
	std::cout << std::endl << "maxVel     :\t\t";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << MaxVel[i] << "\t";
	}
	
	std::cout << std::endl << "fixedVel   :\t\t";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << FixedVel[i] << "\t";
	}

	std::cout << std::endl << "upperLimits:\t\t";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << UpperLimits[i] << "\t";
	}

	std::cout << std::endl << "lowerLimits:\t\t";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << LowerLimits[i] << "\t";
	}
	
	std::cout << std::endl << "offsets    :\t\t";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << Offsets[i] << "\t";
	}

	std::cout << std::endl << "============================================================================== " << std::endl;
	
	/// now open serial port
	int spres = m_sd->openPort(SerialDeviceName.c_str(), SerialBaudrate, 2, 0);
	if(spres == -1)
	{
		errorMsg << "Could not open device " << SerialDeviceName;
		m_ErrorMessage = errorMsg.str();
		return false;	
	}

	/// this is for security. in case the robot is already outputing data, shut it...
	m_sd->PutString("R0\n");	
	if( !m_sd->FlushInBuffer() )
		return false;	/// ...and flush serial port input buffer

//	std::cout << "Detecting the robot..." << std::endl;
	/*m_sd->PutString("N\n");
	std::string str;
	m_sd->GetString(str);
	
	while(str.find("COB3DMD") == std::string::npos)
	{
		m_sd->GetString(str);
		str.resize(30);	// Resize just in case weird name is too long
		errorMsg << "Unknown robot. ID: """ << str.c_str() << std::endl ;
		m_ErrorMessage = errorMsg.str();
		return false;
	}
*/
//	std::cout << "Robot successfuly detected. ID: """ << str.c_str() << """" << std::endl;
	

	m_SerialDeviceOpened = true;

	std::cout << "Now running calibration" << std::endl;
	/// run pan calibration
	if(!RunCalibration())
	{
		errorMsg << "Calibration failed.";
		m_ErrorMessage = errorMsg.str();
		return false;
	}
	
	m_sd->PutString("R1\n");
	m_Initialized = true;
	
	return true;
}

/*
 * \brief Run Calibration
 *
 *
 */
bool MapDemonCtrl::RunCalibration()
{
	std::string str;
	std::ostringstream errorMsg;
	std::vector<std::string> JointNames = m_params_->GetJointNames();
	std::vector<double> MaxVel = m_params_->GetMaxVel();
	
	/// Shut robot output in case it was already enabled...
	m_sd->PutString("R0\n");
	if( !m_sd->FlushInBuffer() )
		return false;	/// ...and flush serial port input buffer
	
	/// Run encoder calibration
	m_sd->PutString("L\n");
	m_sd->GetString(str);		/// lock till getting response
	
	char *cstr = new char (str.size()+1);
	strcpy(cstr, str.c_str());

	if(strchr(cstr, 'L') == NULL)
		return false;

	/// Reposition to Home in case of existant Offsets
	std::vector<double> tmppos( 2, 0.0f );
	MovePos(tmppos);
		
	return true;
}

/*
 * \brief Move both joints to designed position, constant velocity
 *
 * This will use the velocity defined in parameter server as Velocity
 * \param JointNames Vector target_positions Vector
 */
bool MapDemonCtrl::MovePos( const std::vector<double>& target_positions )
{	
	std::vector<double> Offsets = m_params_->GetOffsets();
	std::vector<double> velo = m_params_->GetFixedVels();
	int target_step[2];
	int time_to_target[2];
	int lat[2];
	
	std::stringstream ss;
	
	/// TODO: Mathematically simplify
	target_step[0] = (int)round( (target_positions[0] + Offsets[0]) * 63.66197724f );	/// convert radians to step number, consider offset
	target_step[1] = (int)round( (target_positions[1] + Offsets[1]) * 81.4873308631f );	/// convert radians to step number, consider offset
	
	time_to_target[0] = (int)round(1000 * (target_positions[0] - m_positions[0]) / velo[0] );	// "Interstep latency = delta_position / vel" 
	time_to_target[1] = (int)round(1000 * (target_positions[1] - m_positions[1]) / velo[1] );	// "Interstep latency = delta_position / vel" 
	
	lat[0] = time_to_target[0] / ((target_positions[0] - m_positions[0]) * 63.66197724f) ;
	lat[1] = time_to_target[1] / ((target_positions[1] - m_positions[1]) * 81.4873308631f) ;
	
	printf("TARGET POS: %f\n", target_positions[0]);
	printf("TARGET STEP: %d\n", target_step[0]);
	printf("Lat: %d\n", lat[0]);
	
	ss << "P" << target_step[0] << "," << lat[0] << std::endl;
	ss << "T" << target_step[1] << "," << lat[1] << std::endl << std::ends;

	m_sd->PutString(ss.str());

	return true;
}

/*
 * \brief Move joints with calculated velocities
 *
 * Calculating positions and times by desired value of the cob_trajectory_controller
 * \param velocities Vector
 */
bool MapDemonCtrl::MoveVel(const std::vector<double>& target_velocities)
{
	unsigned int DOF = m_params_->GetDOF();
	
	std::vector<std::string> JointNames = m_params_->GetJointNames();
	std::vector<double> lowerLimits = m_params_->GetLowerLimits();
	std::vector<double> upperLimits = m_params_->GetUpperLimits();
	std::vector<double> Offsets = m_params_->GetOffsets();

	std::ostringstream ss;
	std::ostringstream errorMsg;
	bool errSet = false;
	
	double delta_t;
	double target_pos[2];
	int target_step[2];
	int latencies[2];

	//if ( !UpdatePositions() )
	//	return false;
		
	/// calculate elapsed time since last velocity query for integration
  	delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
  	
  	/// TODO: Glitches here, delta_t sometimes gets a 'weird' value and makes turn the other way of a sudden, decalibrating
  	m_last_time_pub = ros::Time::now();
  	delta_t = delta_t > 0.1 ? 0.1 : delta_t;	// Limit delta_t to avoid insane velocities in case of long delta_t
	
	/// final position in radians
	target_pos[0] = target_velocities[0] * delta_t + m_positions[0] + Offsets[0];
	target_pos[1] = target_velocities[1] * delta_t + m_positions[1] + Offsets[1];

	/// check for out of limit
	for(unsigned int i=0; i<DOF; i++)
	{
		if(target_pos[i] < lowerLimits[i])
		{
			errorMsg << "Computed final position for joint """ << JointNames[i] << """ exceeds limit " << lowerLimits[i] << std::endl;
			m_ErrorMessage = errorMsg.str();
			target_pos[i] = lowerLimits[i];
			m_velocities[0] = 0.0f; 
			errSet = true;
		}
		if(target_pos[i] > upperLimits[i])
		{
			errorMsg << "Computed final position for joint """ << JointNames[i] << """ exceeds limit " << upperLimits[i] << std::endl;
			m_ErrorMessage = errorMsg.str();
			target_pos[i] = upperLimits[i];
			m_velocities[0] = 0.0f;
			errSet = true;
		}
		else
			errSet = false;
	}
	
	/// calculate final position after the single velocity command, in steps (first command after "P")	
	target_step[0] = (int)round( target_pos[0] * 63.66197724f );
	target_step[1] = (int)round( target_pos[1] * 81.4873308631f );
	
	/// calculate steps to jump
	//double stepper_delta_step = fabs(target_velocities[0]) * delta_t / 0.015707963f ;
	//double servo_delta_step = fabs(target_velocities[1]) * delta_t / 0.0122718463f;
	//int stepper_delta_step = (int)round( fabs(target_velocities[0]) * delta_t * 63.66197724f );
	//int servo_delta_step = (int)round( fabs(target_velocities[1]) * delta_t * 81.4873308631f );


	double stepper_delta_step = fabs((target_pos[0] - m_positions[0]) * 63.66197724f) ;
	double servo_delta_step =  fabs((target_pos[1] - m_positions[1]) * 81.4873308631f) ;
	
	/// calculate latency -in ms- for the window
	//latencies[0] = (int)round( 0.015707963 * 1000 / stepper_delta_step ) ;
	//latencies[1] = (int)round( 0.0122718463 * 1000 / servo_delta_step ) ;
	latencies[0] = ( (int)round(delta_t * 1000 / stepper_delta_step ) );
	latencies[1] = ( (int)round(delta_t * 1000 / servo_delta_step ) );
	
	latencies[0] = (latencies[0] > 4) ? latencies[0] : 4;	// security for encoder
	//latencies[1] = (latencies[1] > 1) ? latencies[
	ss << "P" << target_step[0] << "," << latencies[0] << std::endl;
	ss << "T" << target_step[1] << "," << latencies[1] << std::endl;
	
	ss << std::ends;	/// end string with null for the uart driver

	m_sd->PutString(ss.str());
	
	m_velocities[0] = target_velocities[0];
	m_velocities[1] = target_velocities[1];
	
	if(errSet)
		return false;
	
	return true;
}

/*
 * \brief Queries the robot its real position
 *
 * The robot answers p, e, t
 * \param
 */
bool MapDemonCtrl::UpdatePositions()
{
	std::vector<double> Offsets = m_params_->GetOffsets();

	std::ostringstream errorMsg;
	std::string str;
	char *p_pos;
	char *e_pos;
	char *t_pos;
	
	//if( !m_sd->FlushInBuffer() )
	//	return false;	/// ...and flush serial port input buffer
	
	//m_sd->PutString("r\n");
	m_sd->GetString(str);	/// Get next position str

	str.resize(40, NULL);	// max Positions string size is 20 (5x3 ints, 2 commas, 'R' and ':'), resize for security to the double ensuring at least one can be read if present

	char* cstr = new char (str.size()+1);
	strcpy(cstr, str.c_str());	// cstr point to found position

	/// Check for processable string
	p_pos = strchr(cstr,'R')+2;	/// lookup 'R', skip :
	if(p_pos == NULL)
		return false;

	e_pos = strchr(p_pos, ',');	// lookup first ','
	if(e_pos == NULL)
		return false;
	*e_pos = '\0';		// replace comma by null to mark end of first token
	e_pos++;				// advance ptr2 to begining of next token

	t_pos = strchr(e_pos, ',');	// lookup second ','
	if(t_pos == NULL)
		return false;
	*t_pos = '\0';		// replace comma by null to mark end of first token
	t_pos++;				// advance ptr2 to begining of next token

	m_positions[0] = (double)atoi(p_pos) * 0.015707963f - Offsets[0];	// convert step number to angle
	m_encoder = (double)atoi(e_pos) * 0.003141593f - Offsets[0];
	m_positions[1] = (double)atoi(t_pos) * 0.0122718463f - Offsets[1];	// convert servo pos to angle
	
	/// check angular difference between reported step count and encoder position. 
	if(fabs(m_positions[0] - m_encoder) > 0.05f)
	{
		errorMsg << "Pan reported incongruent position. Run recover";
		m_ErrorMessage = errorMsg.str();
		return false;
	}
	else
		m_ErrorMessage = "";
	
	return true;
}

bool MapDemonCtrl::Stop()
{
	std::ostringstream errorMsg;
	std::string str;
	
	m_sd->PutString("E\n");
	m_sd->GetString(str);
	if ( str.find("E\n") == std::string::npos )
	{
		errorMsg << "COB3DMD did not confirm halt";
		m_ErrorMessage = errorMsg.str();
		return false;
	}
	
	return true;
}
/*!
 * \brief Close
 */
bool MapDemonCtrl::Close()
{	
	m_sd->PutString("E\n");	// stop movement
	m_Initialized = false;
	m_sd->closePort();
	m_SerialDeviceOpened = false;
	return true;
}

/*!
 * \brief Recovery after emergency stop or power supply failure
 */
bool MapDemonCtrl::Recover()
{
	std::ostringstream errorMsg;

	if( !m_sd->checkIfStillThere() )
	{
		errorMsg << "COB3DMD not detected anymore.";
		m_sd->closePort();
		m_ErrorMessage = errorMsg.str();
		return false;
	}
	else if(!RunCalibration())
	{
		errorMsg << "COB3DMD recalibration failed.";
		m_ErrorMessage = errorMsg.str();
		return false;
	}
	else
		return true;
}
