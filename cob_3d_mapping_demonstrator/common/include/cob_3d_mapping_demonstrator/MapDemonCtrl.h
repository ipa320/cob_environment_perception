#ifndef __3D_MAP_DEMON_H_
#define __3D_MAP_DEMON_H_

// standard includes
#include <pthread.h>
#include <string>

//own includes
#include <cob_3d_mapping_demonstrator/MapDemonCtrl.h>
#include <cob_3d_mapping_demonstrator/MapDemonCtrlParams.h>
#include <cob_3d_mapping_demonstrator/SerialDevice.h>

class MapDemonCtrl
{
public:

	/// Constructor
	MapDemonCtrl(MapDemonCtrlParams * params);
	MapDemonCtrl(MapDemonCtrlParams * params, SerialDevice * sd);

	/// Destructor
	virtual ~MapDemonCtrl();

	pthread_mutex_t m_mutex;

	virtual bool Init(MapDemonCtrlParams * params);

	virtual bool isInitialized() const
	{
		return m_Initialized;
	}

	virtual bool RunCalibration() ;

	virtual bool MovePos( const std::vector<double>& target_positions );
	virtual bool MoveVel( const std::vector<double>& target_velocities );

	virtual std::string getErrorMessage() const
	{
		return m_ErrorMessage;
	}

	virtual bool Close() ;
	virtual bool Stop();
	virtual bool Recover() ;

	//////////////////////////////////
	// functions to set parameters: //
	//////////////////////////////////

	/*!
	 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
	 *
	 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
	 */
	bool setMaxVelocity(double velocity);
	bool setMaxVelocity(const std::vector<double>& velocities);

	/*!
	 * \brief Gets the current positions
	 */
	virtual std::vector<double> GetPositions()
	{
		return m_positions;
	}

	/*!
	 * \brief Gets the current velcities
	 */
	virtual std::vector<double> GetVelocities()
	{
		return m_velocities;
	}

	virtual bool UpdatePositions();


protected:
	bool m_Initialized;
	int m_DeviceHandle;
	bool m_SerialDeviceOpened;

	MapDemonCtrlParams* m_params_;

	SerialDevice * m_sd;

	std::vector<double> m_positions;
	std::vector<double> m_old_positions;

	std::vector<double> m_velocities;

	double m_encoder;

	ros::Time m_last_time_pub;

	std::string m_ErrorMessage;


};

#endif
