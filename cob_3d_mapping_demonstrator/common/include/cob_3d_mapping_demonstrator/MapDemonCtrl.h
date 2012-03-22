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
	~MapDemonCtrl();

	pthread_mutex_t m_mutex;

	bool Init(MapDemonCtrlParams * params);

	bool isInitialized() const
	{
		return m_Initialized;
	}

	bool RunCalibration() ;

	bool MovePos( const std::vector<double>& target_positions );
	bool MoveVel( const std::vector<double>& target_velocities );

	std::string getErrorMessage() const
	{
		return m_ErrorMessage;
	}

	bool Close() ;
	bool Stop();
	bool Recover() ;

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
	std::vector<double> GetPositions()
	{
		return m_positions;
	}

	/*!
	 * \brief Gets the current velcities
	 */
	std::vector<double> GetVelocities()
	{
		return m_velocities;
	}

	bool UpdatePositions();


private:
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
