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

MapDemonCtrl::MapDemonCtrl(MapDemonCtrlParams * params)
{
  serial_device_opened_ = false;
  initialized_ = false;
  last_time_pub_ = ros::Time::now();
  params_ = params;
  sd_ = new SerialDevice();//sd;
}

/// Destructor
MapDemonCtrl::~MapDemonCtrl()
{
  sd_->PutString("R0\n");      /// shut the robot position output
  sd_->FlushInBuffer();
  sd_->FlushOutBuffer();

  sd_->closePort();
  delete sd_;
}

/*!
 * \brief Initializing
 */
bool MapDemonCtrl::init(MapDemonCtrlParams * params)
{
  /// get serial port configurable parameters
  std::string SerialDeviceName = params_->getSerialDevice();
  int SerialBaudrate = params_->getBaudRate();
  std::ostringstream errorMsg;

  int DOF = params_->getDOF();
  std::vector<std::string> JointNames = params_->getJointNames();
  std::vector<double> MaxVel = params_->getMaxVel();
  std::vector<double> FixedVel = params_->getVels();
  std::vector<double> Offsets = params_->getOffsets();
  std::vector<double> LowerLimits = params_->getLowerLimits();
  std::vector<double> UpperLimits = params_->getUpperLimits();


  positions_.resize(DOF);
  positions_[0] = 0;
  positions_[1] = 0;
  old_positions_.resize(DOF);
  old_positions_[0] = 0;
  old_positions_[1] = 0;
  velocities_.resize(DOF);
  velocities_[0] = 0;
  velocities_[1] = 0;

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
  int spres = sd_->openPort(SerialDeviceName.c_str(), SerialBaudrate, 2, 0);
  if(spres == -1)
  {
    errorMsg << "Could not open device " << SerialDeviceName;
    error_message_ = errorMsg.str();
    return false;
  }

  /// this is for security. in case the robot is already outputing data, shut it...
  sd_->PutString("R0\n");
  usleep(200000);
  if( !sd_->FlushInBuffer() )
    return false;	/// ...and flush serial port input buffer
  usleep(200000);

  std::cout << "Detecting the robot..." << std::endl;
  //m_sd->PutString("N\n");
  //usleep(200000);
  std::string str("");
  //m_sd->GetString(str);
  sd_->PutString("N\n");

  unsigned int ctr=0;
  while(str.find("COB3DMD") == std::string::npos)
  {
    str.clear();
    sd_->GetString(str);
    std::cout << "return 1: " << str.c_str() << std::endl;
    //str.resize(30);	// Resize just in case weird name is too long
    ctr++;
    if(ctr>=100)
    {
      errorMsg << "Unknown robot. ID: """ << str.c_str() << std::endl ;
      error_message_ = errorMsg.str();
      return false;
    }
  }

  std::cout << "Robot successfuly detected. ID: """ << str.c_str() << """" << std::endl;


  serial_device_opened_ = true;

  std::cout << "Now running calibration" << std::endl;
  /// run pan calibration
  if(!runCalibration())
  {
    errorMsg << "Calibration failed.";
    error_message_ = errorMsg.str();
    return false;
  }

  sd_->PutString("R1\n");
  initialized_ = true;

  return true;
}

/*
 * \brief Run Calibration
 *
 *
 */
bool MapDemonCtrl::runCalibration()
{
  std::ostringstream errorMsg;
  std::vector<std::string> JointNames = params_->getJointNames();
  std::vector<double> MaxVel = params_->getMaxVel();

  /// Shut robot output in case it was already enabled...
  usleep(200000);
  if( !sd_->FlushInBuffer() )
    return false;	/// ...and flush serial port input buffer
  usleep(200000);

  /// Run encoder calibration
  sd_->PutString("L\n");

  size_t found = std::string::npos;
  char retry = 0;	//

  /// get messages till 'L' is received (this is necessary because if the message output of the robot was enabled, between the flush buffer and this we still could have received a few characters from that, before the 'L'.
  while( found == std::string::npos )
  {
    std::string str;
    sd_->GetString(str);	// read another message
    found = str.find_first_of("L", 0);	// find L
    retry++;
    std::cout << "calib cb: " << str << std::endl;
    if(retry == 4)	// experimental value, normally one or two messages are received btw flushinbuffer and getstring.
      return false;	/// if tryout
  }

  //char *cstr = new char (str.size()+1);
  //strcpy(cstr, str.c_str());

  //if(strchr(cstr, 'L') == NULL)
  //	return false;

  /// Reposition to Home in case of existant Offsets
  std::vector<double> tmppos( 2, 0.0f );
  movePos(tmppos);

  return true;
}

/*
 * \brief Move both joints to designed position, constant velocity
 *
 * This will use the velocity defined in parameter server as Velocity
 * \param JointNames Vector target_positions Vector
 */
bool MapDemonCtrl::movePos( const std::vector<double>& target_positions )
{
  std::vector<double> Offsets = params_->getOffsets();
  std::vector<double> velo = params_->getVels();
  int target_step[2];
  int time_to_target[2];
  int lat[2];

  std::stringstream ss;

  /// TODO: Mathematically simplify
  target_step[0] = (int)round( (target_positions[0] + Offsets[0]) * 63.66197724f );	/// convert radians to step number, consider offset
  target_step[1] = (int)round( (target_positions[1] + Offsets[1]) * 81.4873308631f );	/// convert radians to step number, consider offset

  time_to_target[0] = (int)round(1000 * (target_positions[0] - positions_[0]) / velo[0] );	// "Interstep latency = delta_position / vel"
  time_to_target[1] = (int)round(1000 * (target_positions[1] - positions_[1]) / velo[1] );	// "Interstep latency = delta_position / vel"

  lat[0] = time_to_target[0] / ((target_positions[0] - positions_[0]) * 63.66197724f) ;
  lat[1] = time_to_target[1] / ((target_positions[1] - positions_[1]) * 81.4873308631f) ;

  printf("TARGET POS: %f\n", target_positions[0]);
  printf("TARGET STEP: %d\n", target_step[0]);
  printf("Lat: %d\n", lat[0]);

  ss << "P" << target_step[0] << "," << lat[0] << std::endl;
  ss << "T" << target_step[1] << "," << lat[1] << std::endl << std::ends;

  sd_->PutString(ss.str());

  return true;
}

/*
 * \brief Move joints with calculated velocities
 *
 * Calculating positions and times by desired value of the cob_trajectory_controller
 * \param velocities Vector
 */
/*bool MapDemonCtrl::MoveVel(const std::vector<double>& target_velocities)
{
  m_velocities[0] = target_velocities[0];
  m_velocities[1] = target_velocities[1];

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
  delta_t = 0.05;//delta_t > 0.1 ? 0.1 : delta_t;	// Limit delta_t to avoid insane velocities in case of long delta_t

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
  std::cout << "t_step: " << target_step[1] << std::endl;

  /// calculate steps to jump
  //double stepper_delta_step = fabs(target_velocities[0]) * delta_t / 0.015707963f ;
  //double servo_delta_step = fabs(target_velocities[1]) * delta_t / 0.0122718463f;
  //int stepper_delta_step = (int)round( fabs(target_velocities[0]) * delta_t * 63.66197724f );
  //int servo_delta_step = (int)round( fabs(target_velocities[1]) * delta_t * 81.4873308631f );


  double stepper_delta_step = fabs((target_pos[0] - m_positions[0]) * 63.66197724f) ;
  double servo_delta_step =  fabs((target_pos[1] - m_positions[1]) * 81.4873308631f) ;

  if(stepper_delta_step>=1)
  {
    latencies[0] = ( (int)round(delta_t * 1000 / stepper_delta_step ) );
    latencies[0] = (latencies[0] > 4) ? latencies[0] : 4; // security for encoder
    ss << "P" << target_step[0] << "," << latencies[0] << std::endl;
  }

  /// calculate latency -in ms- for the window
  //latencies[0] = (int)round( 0.015707963 * 1000 / stepper_delta_step ) ;
  //latencies[1] = (int)round( 0.0122718463 * 1000 / servo_delta_step ) ;

  std::cout << servo_delta_step << std::endl;
  if(servo_delta_step>=1)
  {
    latencies[1] = ( (int)round(delta_t * 1000 / servo_delta_step ) );
    //latencies[1] = (latencies[1] > 1) ? latencies[
    ss << "T" << target_step[1] << "," << latencies[1] << std::endl;
    std::cout << ss.str() << std::endl;
  }

  ss << std::ends;	/// end string with null for the uart driver

  m_sd->PutString(ss.str());

  if(errSet)
    return false;

  return true;
}*/

/*
 * \brief Queries the robot its real position
 *
 * The robot answers p, e, t
 * \param
 */
bool MapDemonCtrl::updatePositions()
{
  std::vector<double> Offsets = params_->getOffsets();

  std::ostringstream errorMsg;
  std::string str;
  char *p_pos;
  char *e_pos;
  char *t_pos;

  //if( !m_sd->FlushInBuffer() )
  //	return false;	/// ...and flush serial port input buffer

  //m_sd->PutString("r\n");
  sd_->GetString(str);	/// Get next position str

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

  old_positions_[0] = positions_[0];
  old_positions_[1] = positions_[1];
  positions_[0] = (double)atoi(p_pos) * 0.015707963f - Offsets[0];	// convert step number to angle
  encoder_ = (double)atoi(e_pos) * 0.003141593f - Offsets[0];
  positions_[1] = (double)atoi(t_pos) * 0.0122718463f - Offsets[1];	// convert servo pos to angle

  /// check angular difference between reported step count and encoder position.
  if(fabs(positions_[0] - encoder_) > 0.05f)
  {
    errorMsg << "Pan reported incongruent position. Run recover";
    error_message_ = errorMsg.str();
    return false;
  }
  else
    error_message_ = "";

  return true;
}

bool MapDemonCtrl::stop()
{
  std::ostringstream errorMsg;
  std::string str("");

  sd_->PutString("E\n");
  unsigned int ctr=0;
  while(str.find("E") == std::string::npos)
  {
    str.clear();
    sd_->GetString(str);
    if(ctr>50)
    {
      errorMsg << "COB3DMD did not confirm halt";
      error_message_ = errorMsg.str();
      return false;
    }
    ctr++;
  }
  /*if ( str.find("E\n") == std::string::npos )
  {
    errorMsg << "COB3DMD did not confirm halt";
    m_ErrorMessage = errorMsg.str();
    return false;
  }*/

  return true;
}
/*!
 * \brief Close
 */
bool MapDemonCtrl::close()
{
  sd_->PutString("E\n");	// stop movement
  initialized_ = false;
  sd_->closePort();
  serial_device_opened_ = false;
  return true;
}

/*!
 * \brief Recovery after emergency stop or power supply failure
 */
bool MapDemonCtrl::recover()
{
  std::ostringstream errorMsg;

  sd_->PutString("R0\n");	/// shut reposition messages from the robot

  if( !sd_->checkIfStillThere() )
  {
    errorMsg << "COB3DMD not detected anymore.";
    sd_->closePort();
    error_message_ = errorMsg.str();
    return false;
  }
  else if( !runCalibration() )
  {
    errorMsg << "COB3DMD recalibration failed.";
    error_message_ = errorMsg.str();
    return false;
  }
  else
  {
    sd_->PutString("R1\n");
    return true;
  }
}
