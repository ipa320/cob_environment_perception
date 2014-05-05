#ifndef __3D_MAP_DEMON_PARAMS_H_
#define __3D_MAP_DEMON_PARAMS_H_

class DemonstratorParams
{
public:
	/// Constructor
	DemonstratorParams()
	{

	}

	///Destructor
	~DemonstratorParams();

	///Initilizing
	void init(std::string SerialDevice, int BaudRate)
	{
		setSerialDevice(SerialDevice);
		setBaudRate(BaudRate);
	}

	///Sets the name of serial device
	void setSerialDevice(std::string SerialDeviceName) {
		serial_device_name_ = SerialDeviceName;
	}
	///Gets the name of the serial device
	std::string getSerialDevice() {
		return serial_device_name_;
	}

	///Sets the BaudRate
	void setBaudRate(int BaudRate) {
		baud_rate_ = BaudRate;
	}
	///Gets the BaudRate
	int getBaudRate() {
		return baud_rate_;
	}

	///Sets the operation mode
	/*void SetOperationMode(std::string opMode) {
		m_operationMode = opMode;
	}
	///Gets the operation mode
	std::string GetOperationMode() {
		return m_operationMode;
	}*/

	/// Sets the DOF value
	void setDOF(int DOF) {
		dof_ = DOF;
	}

	/// Gets the DOF value
	unsigned int getDOF() {
		return dof_;
	}

	/// Sets the joint names
	void setJointNames(std::vector<std::string> JointNames) {
		joint_names_ = JointNames;
	}
	/// Gets the joint names
	std::vector<std::string> getJointNames() {
		return joint_names_;
	}

	/// Sets the upper angular limits (rad) for the joints
	void setUpperLimits(std::vector<double> UpperLimits) {
		upper_limits_ = UpperLimits;
	}
	/// Gets the upper angular limits (rad) for the joints
	std::vector<double> getUpperLimits() {
		return upper_limits_;
	}

	/// Sets the lower angular limits (rad) for the joints
	void setLowerLimits(std::vector<double> LowerLimits) {
		lower_limits_ = LowerLimits;
	}
	/// Gets the lower angular limits (rad) for the joints
	std::vector<double> getLowerLimits() {
		return lower_limits_;
	}

	/// Sets the max. angular velocities (rad/s) for the joints
	void setMaxVel(std::vector<double> MaxVel) {
		max_vel_ = MaxVel;
	}
	/// Gets the max. angular velocities
	std::vector<double> getMaxVel()
	{
		return max_vel_;
	}

	/// Gets the offsets
	void setOffsets(std::vector<double> Offsets)	{
		offsets_ = Offsets;
	}
	/// Sets the offsets
	std::vector<double> getOffsets() {
		return offsets_;
	}

	/// Gets the fixed velocities
	void setVels( std::vector<double> velocities )	{
		vels_ = velocities;
	}
	/// Sets the offsets
	std::vector<double> getVels() {
		return vels_;
	}

        /// Gets the fixed velocities
        void setAccels( std::vector<double> accels )  {
                accels_ = accels;
        }
        /// Sets the offsets
        std::vector<double> getAccels() {
                return accels_;
        }

private:
	std::string serial_device_name_;
	int baud_rate_;
	//std::string m_operationMode;

	unsigned int dof_;
	std::vector<std::string> joint_names_;
	std::vector<double> upper_limits_;
	std::vector<double> lower_limits_;
	std::vector<double> max_vel_;
	std::vector<double> vels_;
	std::vector<double> accels_;
	std::vector<double> offsets_;
};

#endif

