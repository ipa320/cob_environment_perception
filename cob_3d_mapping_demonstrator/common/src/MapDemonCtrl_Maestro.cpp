// ROS includes
#include <ros/ros.h>

// standard includes
#include <boost/thread.hpp>
#include <string>
#include <cstring>
#include <stdlib.h>
#include <math.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

// own includes
#include <cob_3d_mapping_demonstrator/MapDemonCtrl_Maestro.h>

MapDemonCtrl_Maestro::MapDemonCtrl_Maestro(MapDemonCtrlParams * params, SerialDevice * sd):
MapDemonCtrl(params, sd), STEP_WIDTH(255/(2*M_PI))
{
}

/// Destructor
MapDemonCtrl_Maestro::~MapDemonCtrl_Maestro()
{
}

/*!
 * \brief Initializing
 */
bool MapDemonCtrl_Maestro::Init(MapDemonCtrlParams * params)
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
  m_positions[0] = 0;
  m_positions[1] = 0;
  m_old_positions.resize(DOF);
  m_old_positions[0] = 0;
  m_old_positions[1] = 0;
  m_velocities.resize(DOF);
  m_velocities[0] = 0;
  m_velocities[1] = 0;

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

  m_fd =  open(SerialDeviceName.c_str(), O_RDWR | O_NOCTTY);

  /// open(2) returns <0 if the port could NOT be opened
  if (m_fd == -1 ) {
    errorMsg << "Could not open device " << SerialDeviceName;
    m_ErrorMessage = errorMsg.str();
    return false;
  }

  struct termios options;
  tcgetattr(m_fd, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(m_fd, TCSANOW, &options);

  usleep(10000);

  if(!UpdatePositions()) return false;

  setMaxVelocity(0.1);
  m_Initialized = true;

  return true;
}

/*
 * \brief Run Calibration
 *
 *
 */
bool MapDemonCtrl_Maestro::RunCalibration()
{
  return true;
}

/*
 * \brief Move both joints to designed position, constant velocity
 *
 * This will use the velocity defined in parameter server as Velocity
 * \param JointNames Vector target_positions Vector
 */
bool MapDemonCtrl_Maestro::MovePos( const std::vector<double>& target_positions )
{
  int DOF = m_params_->GetDOF();

  for(int i=0; i<DOF; i++) {
    int pos = rad2int(target_positions[i], i);
    unsigned char list[2] = {(unsigned short)pos & 0x7F, (unsigned short)(pos >> 7) & 0x7F};
    writeCmd(SET_TARGET, i, list, 2);
  }

  return true;
}

/*
 * \brief Move joints with calculated velocities
 *
 * Calculating positions and times by desired value of the cob_trajectory_controller
 * \param velocities Vector
 */
bool MapDemonCtrl_Maestro::MoveVel(const std::vector<double>& target_velocities)
{
  ROS_ERROR("not supported yet");
  return false;
}

/*
 * \brief Queries the robot its real position
 *
 * The robot answers p, e, t
 * \param
 */
bool MapDemonCtrl_Maestro::UpdatePositions()
{
  int DOF = m_params_->GetDOF();

  for(int i=0; i<DOF; i++) {
    writeCmd(GET_POSITION, i);

    std::string str("");
    unsigned int ctr=0;
    while(str.size()<2)
    {
      char buf[255];
      size_t nbytes;

      nbytes = read(m_fd, buf, 255);
      str += std::string(buf, nbytes);

//      std::cout<<"recv: ";
//      for(int i=0; i<str.size(); i++)
//        printf("%x ",(int)(unsigned char)str[i]);
//      std::cout<<"\n";

      ++ctr;
      usleep(10000);

      if(ctr>100) {
        ROS_ERROR("failed to connect to device");
        return false;
      }
    }
    m_positions[i] = int2rad((unsigned char)str[0] + ((unsigned char)str[1])*256);
  }

//  std::vector<double> p;
//  p.push_back(100);
//  p.push_back(100);
//  MovePos(p);

  return true;
}

bool MapDemonCtrl_Maestro::Stop()
{
  ROS_ERROR("not implemented");

  return true;
}
/*!
 * \brief Close
 */
bool MapDemonCtrl_Maestro::Close()
{
  m_Initialized = false;
  m_sd->closePort();
  m_SerialDeviceOpened = false;
  return true;
}

/*!
 * \brief Recovery after emergency stop or power supply failure
 */
bool MapDemonCtrl_Maestro::Recover()
{
  std::ostringstream errorMsg;

  if( !m_sd->checkIfStillThere() )
  {
    errorMsg << "COB3DMD not detected anymore.";
    m_sd->closePort();
    m_ErrorMessage = errorMsg.str();
    return false;
  }
}

bool MapDemonCtrl_Maestro::setMaxVelocity(double velocity)
{
  int DOF = m_params_->GetDOF();
  for(int i=0; i<DOF; i++) {
    unsigned int v = 50;
    unsigned char vel[2] = {(unsigned short)v & 0x7F, (unsigned short)(v >> 7) & 0x7F};
    writeCmd(SET_VEL, i, vel, 2);
  }
}

void MapDemonCtrl_Maestro::writeCmd(const unsigned char cmd, const unsigned char channel, const unsigned char *data, const int size)
{
  ROS_ASSERT(m_sd);
  std::string s;
  s.push_back(cmd);
  s.push_back(channel);
  for(int i=0; i<size; i++)
    s.push_back(data[i]);

//  std::cout<<"writing: ";
//  for(int i=0; i<s.size(); i++)
//    printf("%x ",(int)(unsigned char)s[i]);
//  std::cout<<"\n";

  if(write(m_fd, s.c_str(), s.size())!=s.size())
    ROS_WARN("could not send to serial");
}
