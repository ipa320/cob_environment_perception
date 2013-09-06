/*
 * MapDemonCtrl_Maestro.h
 *
 *  Created on: 16.11.2012
 *      Author: josh
 */

#ifndef MAPDEMONCTRL_MAESTRO_H_
#define MAPDEMONCTRL_MAESTRO_H_

#include "cob_3d_mapping_demonstrator/demonstrator_control.h"

class MapDemonCtrlMaestro : public MapDemonCtrl
{
public:

  /// Constructor
  MapDemonCtrlMaestro(DemonstratorParams * params);

  /// Destructor
  virtual ~MapDemonCtrlMaestro();

  virtual bool init(DemonstratorParams * params);

  virtual bool runCalibration();

  virtual bool movePos( const std::vector<double>& target_positions );
  //virtual bool MoveVel( const std::vector<double>& target_velocities );

  virtual bool close() ;
  virtual bool stop();
  virtual bool recover();

  virtual void setVelocity();
  virtual void setAcceleration();

  virtual bool updatePositions();
  virtual bool isMoving();

  bool is_moving_;

private:

  enum {
    GET_POSITION=0x90,
    SET_TARGET=0x84,
    SET_VEL=0x87,
    SET_ACCEL=0x89,
    IS_MOVING=0x93,
    GO_HOME=0xA2
  };

  // Handle for serial device
  int fd_;

  //const double STEP_WIDTH;

  void writeCmd(const unsigned char cmd, const unsigned char channel=255, const unsigned char *data=NULL, const int size=0);

  int rad2int(const double v, const int dof = 0) {
    double rad = v;
    //rad+=M_PI;
    //if(rad<0) rad = 2*M_PI+rad;
    return round((1/M_PI*rad*1800 + 1500)*4);
    //return round( 2*4900*rad/(2*M_PI) + 1000);
    //return round( (v-m_params_->GetOffsets()[dof])*STEP_WIDTH);
  }

  double int2rad(const int v, const int dof = 0) {
    return (v/4-1500)*M_PI/1800;
    //return (v-1000)*(2*M_PI)/(2*4900) - M_PI;
    //return (v/STEP_WIDTH)+m_params_->GetOffsets()[dof];
  }
};

#endif /* MAPDEMONCTRL_MAESTRO_H_ */
