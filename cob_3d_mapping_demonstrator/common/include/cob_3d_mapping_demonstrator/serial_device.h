#ifndef INC_SERIALPORTDEVICE_H
#define INC_SERIALPORTDEVICE_H

#include <string>

class SerialDevice
{
public:

	SerialDevice();

	~SerialDevice();

	int openPort(std::string , int , int, int );

	void closePort();

	bool FlushInBuffer();
	bool FlushOutBuffer();

	int PutString(std::string );

	bool checkIfStillThere();

	void GetString( std::string& rxstr );
	void GetStringWithEndline( std::string& rxstr );

	unsigned char GetChar();

private:
	int fd_;
};

#endif
