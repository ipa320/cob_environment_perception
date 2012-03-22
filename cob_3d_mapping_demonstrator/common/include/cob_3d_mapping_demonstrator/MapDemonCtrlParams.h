#ifndef __3D_MAP_DEMON_PARAMS_H_
#define __3D_MAP_DEMON_PARAMS_H_

class MapDemonCtrlParams
{
public:
	/// Constructor
	MapDemonCtrlParams()
	{

	}
	
	///Destructor
	~MapDemonCtrlParams();
	
	///Initilizing
	void Init(std::string SerialDevice, int BaudRate)
	{
		SetSerialDevice(SerialDevice);
		SetBaudRate(BaudRate);		
	}
	
	///Sets the name of serial device
	void SetSerialDevice(std::string SerialDeviceName) {
		m_SerialDeviceName = SerialDeviceName;
	}	
	///Gets the name of the serial device
	std::string GetSerialDevice() {
		return m_SerialDeviceName;
	}
	
	///Sets the BaudRate
	void SetBaudRate(int BaudRate) {
		m_BaudRate = BaudRate;
	}	
	///Gets the BaudRate
	int GetBaudRate() {
		return m_BaudRate;
	}
	
	///Sets the operation mode
	void SetOperationMode(std::string opMode) {
		m_operationMode = opMode;
	}
	///Gets the operation mode
	std::string GetOperationMode() {
		return m_operationMode;
	}
	
	/// Sets the DOF value
	void SetDOF(int DOF) {
		m_DOF = DOF;
	}

	/// Gets the DOF value
	unsigned int GetDOF() {
		return m_DOF;
	}
	
	/// Sets the joint names
	void SetJointNames(std::vector<std::string> JointNames) {
		m_JointNames = JointNames;
	}
	/// Gets the joint names
	std::vector<std::string> GetJointNames() {
		return m_JointNames;
	}
	
	/// Sets the upper angular limits (rad) for the joints
	void SetUpperLimits(std::vector<double> UpperLimits) {
		m_UpperLimits = UpperLimits;
	}
	/// Gets the upper angular limits (rad) for the joints
	std::vector<double> GetUpperLimits() {
		return m_UpperLimits;
	}
		
	/// Sets the lower angular limits (rad) for the joints
	void SetLowerLimits(std::vector<double> LowerLimits) {
		m_LowerLimits = LowerLimits;
	}
	/// Gets the lower angular limits (rad) for the joints
	std::vector<double> GetLowerLimits() {
		return m_LowerLimits;
	}
	
	/// Sets the max. angular velocities (rad/s) for the joints
	void SetMaxVel(std::vector<double> MaxVel) {
		m_MaxVel = MaxVel;
	}
	/// Gets the max. angular velocities
	std::vector<double> GetMaxVel()
	{
		return m_MaxVel;
	}
	
	/// Gets the offsets
	void SetOffsets(std::vector<double> Offsets)	{
		m_Offsets = Offsets;
	}
	/// Sets the offsets
	std::vector<double> GetOffsets() {
		return m_Offsets;
	}

	/// Gets the fixed velocities
	void SetFixedVels( std::vector<double> fixedVelocities )	{
		m_fixedVels = fixedVelocities;
	}
	/// Sets the offsets
	std::vector<double> GetFixedVels() {
		return m_fixedVels;
	}
	
private:
	std::string m_SerialDeviceName;
	int m_BaudRate;
	std::string m_operationMode;
	
	unsigned int m_DOF;
	std::vector<std::string> m_JointNames;
	std::vector<double> m_UpperLimits;
	std::vector<double> m_LowerLimits;
	std::vector<double> m_MaxVel;
	std::vector<double> m_fixedVels;
	std::vector<double> m_Offsets;
};

#endif

