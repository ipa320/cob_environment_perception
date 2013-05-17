/*
 * MapDemonCtrl_Maestro.h
 *
 *  Created on: 16.11.2012
 *      Author: josh
 */

#ifndef MAPDEMONCTRL_MAESTRO_H_
#define MAPDEMONCTRL_MAESTRO_H_

#include "MapDemonCtrl.h"

class MapDemonCtrl_Maestro : public MapDemonCtrl
{
public:

  /// Constructor
  MapDemonCtrl_Maestro(MapDemonCtrlParams * params, SerialDevice * sd);

  /// Destructor
  virtual ~MapDemonCtrl_Maestro();

  virtual bool Init(MapDemonCtrlParams * params);

  virtual bool RunCalibration() ;

  virtual bool MovePos( const std::vector<double>& target_positions );
  virtual bool MoveVel( const std::vector<double>& target_velocities );

  virtual bool Close() ;
  virtual bool Stop();
  virtual bool Recover() ;

  virtual bool UpdatePositions();

private:

  enum {
    GET_POSITION=0x90,
    SET_TARGET=0x84
  };

  int m_fd;

  const double STEP_WIDTH;

  void writeCmd(const unsigned char cmd, const unsigned char channel, const unsigned char *data=NULL, const int size=0);

  int rad2int(const double v, const int dof) {
    double rad = v;
    rad+=M_PI;
    if(rad<0) rad = 2*M_PI+rad;
    return round( 2*4900*rad/(2*M_PI) + 1000);
    //return round( (v-m_params_->GetOffsets()[dof])*STEP_WIDTH);
  }

  int rad2int(const int v, const int dof) {
    return v-1000*(2*M_PI)/(2*4900) - M_PI;
    //return (v/STEP_WIDTH)+m_params_->GetOffsets()[dof];
  }
};

#endif /* MAPDEMONCTRL_MAESTRO_H_ */
