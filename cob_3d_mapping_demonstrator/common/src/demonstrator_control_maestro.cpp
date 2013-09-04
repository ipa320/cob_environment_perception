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
#include <cob_3d_mapping_demonstrator/demonstrator_control_maestro.h>

MapDemonCtrlMaestro::MapDemonCtrlMaestro(DemonstratorParams * params):
MapDemonCtrl(params)
{
}

/// Destructor
MapDemonCtrlMaestro::~MapDemonCtrlMaestro()
{
}

/*!
 * \brief Initializing
 */
bool MapDemonCtrlMaestro::init(DemonstratorParams * params)
{
  /// get serial port configurable parameters
  //std::string SerialDeviceName = m_params_->GetSerialDevice();
  //int SerialBaudrate = m_params_->GetBaudRate();
  std::ostringstream errorMsg;

  int DOF = params_->getDOF();
  positions_.resize(DOF, 0);
  old_positions_.resize(DOF, 0);
  velocities_.resize(DOF, 0);


  std::vector<std::string> JointNames = params_->getJointNames();
  std::vector<double> MaxVel = params_->getMaxVel();
  std::vector<double> FixedVel = params_->getVels();
  std::vector<double> Offsets = params_->getOffsets();
  std::vector<double> LowerLimits = params_->getLowerLimits();
  std::vector<double> UpperLimits = params_->getUpperLimits();


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

  fd_ = open(params_->getSerialDevice().c_str(), O_RDWR | O_NOCTTY);

  /// open(2) returns <0 if the port could NOT be opened
  if (fd_ == -1 ) {
    errorMsg << "Could not open device " << params_->getSerialDevice();
    error_message_ = errorMsg.str();
    return false;
  }

  struct termios options;
  tcgetattr(fd_, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(fd_, TCSANOW, &options);

  usleep(500000);

  //if(!updatePositions()) return false;

  setVelocity();
  setAcceleration();
  //writeCmd(GO_HOME);
  std::vector<double> target_positions(2,0);
  movePos(target_positions);
  initialized_ = true;

  return true;
}

/*
 * \brief Run Calibration
 *
 *
 */
bool MapDemonCtrlMaestro::runCalibration()
{
  return true;
}

/*
 * \brief Move both joints to designed position, constant velocity
 *
 * This will use the velocity defined in parameter server as Velocity
 * \param JointNames Vector target_positions Vector
 */
bool MapDemonCtrlMaestro::movePos( const std::vector<double>& target_positions )
{
  int DOF = params_->getDOF();

  for(int i=0; i<DOF; i++) {
    if(target_positions[i] > params_->getUpperLimits()[i] || target_positions[i] < params_->getLowerLimits()[i])
    {
      std::stringstream error_msg;
      error_msg << "Position " << target_positions[i] << " exceeds limit for axis " << i;
      error_message_ = error_msg.str();
      return false;
    }
    int pos = rad2int(target_positions[i], i);
    //ROS_INFO("move pos %f", target_positions[i]);
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
/*bool MapDemonCtrlMaestro::MoveVel(const std::vector<double>& target_velocities)
{
  ROS_ERROR("not supported yet");
  return false;
}*/

/*
 * \brief Queries the robot its real position
 *
 * The robot answers p, e, t
 * \param
 */
bool MapDemonCtrlMaestro::updatePositions()
{
  int DOF = params_->getDOF();

  for(int i=0; i<DOF; i++) {
    writeCmd(GET_POSITION, i);

    //std::string str("");

    unsigned char buf[2];
    while(read(fd_,buf,sizeof(buf)) != sizeof(buf))
    {
      usleep(1000);
      ROS_ERROR("error reading");
      //return false;
    }
    /*unsigned int ctr=0;
    while(str.size()<2)
    {
      char buf[255];
      size_t nbytes;

      nbytes = read(fd_, buf, 255);
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
    }*/
    positions_[i] = int2rad(buf[0] | buf[1]<<8/*(unsigned char)str[0] + ((unsigned char)str[1])*256*/);
  }
  is_moving_ = isMoving();
  return true;
}

bool MapDemonCtrlMaestro::isMoving()
{
  //int DOF = params_->getDOF();

  //for(int i=0; i<DOF; i++) {
    writeCmd(IS_MOVING);

    //std::string str("");

    unsigned char buf[1];
    while(read(fd_,buf,sizeof(buf)) != sizeof(buf))
    {
      usleep(1000);
      ROS_ERROR("error reading");
    }
    //std::cout << "isM: " << (unsigned int)buf[0] << std::endl;
    if((unsigned int)buf[0] == 1) return true;
    else return false;
  //}

  return true;
}

bool MapDemonCtrlMaestro::stop()
{
  //updatePositions();
  //ROS_INFO("pos: %f, %f", positions_[0], positions_[1]);
  movePos(positions_);
  //ROS_ERROR("not implemented");
  return true;
}
/*!
 * \brief Close
 */
bool MapDemonCtrlMaestro::close()
{
  /*initialized_ = false;
  sd_->closePort();
  serial_device_opened_ = false;*/
  return true;
}

/*!
 * \brief Recovery after emergency stop or power supply failure
 */
bool MapDemonCtrlMaestro::recover()
{
  /*std::ostringstream errorMsg;

  if( !sd_->checkIfStillThere() )
  {
    errorMsg << "COB3DMD not detected anymore.";
    sd_->closePort();
    error_message_ = errorMsg.str();
    return false;
  }*/
  return true;
}

void MapDemonCtrlMaestro::setVelocity()
{
  int DOF = params_->getDOF();
  for(int i=0; i<DOF; i++) {
    unsigned int v = params_->getVels()[i]*72;
    //ROS_INFO("Setting vel to %d.", v);
    unsigned char vel[2] = {(unsigned short)v & 0x7F, (unsigned short)(v >> 7) & 0x7F};
    writeCmd(SET_VEL, i, vel, 2);
  }
}

void MapDemonCtrlMaestro::setAcceleration()
{
  int DOF = params_->getDOF();
  for(int i=0; i<DOF; i++) {
    unsigned int v = params_->getAccels()[i]*228.6;
    ROS_INFO("Setting accel to %d.", v);
    unsigned char accel[2] = {(unsigned short)v & 0x7F, (unsigned short)(v >> 7) & 0x7F};
    writeCmd(SET_ACCEL, i, accel, 2);
  }
}

void MapDemonCtrlMaestro::writeCmd(const unsigned char cmd, const unsigned char channel, const unsigned char *data, const int size)
{
  //ROS_ASSERT(sd_);
  std::string s;
  s.push_back(cmd);
  if( channel != 255)
    s.push_back(channel);
  for(int i=0; i<size; i++)
    s.push_back(data[i]);


  /*std::cout<<"writing: ";
  for(unsigned int i=0; i<s.size(); i++)
    printf("%x ",(int)(unsigned char)s[i]);
  std::cout<<"\n";*/

  if(write(fd_, s.c_str(), s.size())!=s.size())
    ROS_WARN("could not send to serial");
}
