#ifndef __3D_MAP_DEMON_H_
#define __3D_MAP_DEMON_H_

// standard includes
#include <pthread.h>
#include <string>

//own includes
//#include <cob_3d_mapping_demonstrator/MapDemonCtrl.h>
#include <cob_3d_mapping_demonstrator/demonstrator_params.h>
#include <cob_3d_mapping_demonstrator/serial_device.h>

class MapDemonCtrl
{
public:

  /// Constructor
  MapDemonCtrl(DemonstratorParams * params);
  //MapDemonCtrl(MapDemonCtrlParams * params, SerialDevice * sd);

  /// Destructor
  virtual ~MapDemonCtrl();

  pthread_mutex_t m_mutex;

  virtual bool init(DemonstratorParams * params);

  virtual bool isInitialized() const
  {
    return initialized_;
  }

  virtual bool runCalibration() ;

  virtual bool movePos( const std::vector<double>& target_positions );
  //virtual bool MoveVel( const std::vector<double>& target_velocities );

  virtual std::string getErrorMessage() const
  {
    return error_message_;
  }

  virtual bool close() ;
  virtual bool stop();
  virtual bool recover() ;

  //////////////////////////////////
  // functions to set parameters: //
  //////////////////////////////////

  /*!
   * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
   *
   * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
   */
  virtual void setVelocity() {};
  bool setMaxVelocity(const std::vector<double>& velocities);

  /*!
   * \brief Gets the current positions
   */
  virtual std::vector<double> getPositions()
	    {
    return positions_;
	    }

  /*!
   * \brief Gets the current velcities
   */
  virtual std::vector<double> getVelocities()
	    {
    return velocities_;
	    }

  virtual bool updatePositions();


protected:
  bool initialized_;
  int device_handle_;
  bool serial_device_opened_;

  DemonstratorParams* params_;

  SerialDevice * sd_;

  std::vector<double> positions_;
  std::vector<double> old_positions_;

  std::vector<double> velocities_;

  double encoder_;

  ros::Time last_time_pub_;

  std::string error_message_;


};

#endif
