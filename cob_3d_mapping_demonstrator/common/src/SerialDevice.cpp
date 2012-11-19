/// standard includes
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>

#include <string>
#include <iostream>

/// self includes
#include <cob_3d_mapping_demonstrator/SerialDevice.h>

SerialDevice::SerialDevice()
{

}

SerialDevice::~SerialDevice()
{
 /*std::cout << "destructor" << std::endl;
  PutString("L\n");
  FlushInBuffer();
  FlushOutBuffer();
  closePort();*/
}

int SerialDevice::openPort(std::string device, int baud, int Parity, int StopBits)
{
	int BAUD, DATABITS, STOPBITS, PARITYON, PARITY;
	struct termios config;

	// Adapt baud to termios.h baudrate enum
	switch(baud)
	{
		case 57600:
		BAUD = B57600;
		break;
		case 38400:	
		BAUD = B38400;
		break;
		case 19200:
		BAUD  = B19200;
		break;	
		case 9600:
		default:		//If incorrect value is entered, baud will be defaulted to 9600
		BAUD  = B9600;
		break;
	}
	switch (Parity)
	{
		case 0:
		default:                       //none
		PARITYON = 0;
		PARITY = 0;
		break;
		case 1:                        //odd
		PARITYON = PARENB;
		PARITY = PARODD;
		break;
		case 2:                        //even
		PARITYON = PARENB;
		PARITY = 0;
		break;
	}
	device = device.insert(0,"/dev/");	/// inserts '/dev/' infront of device name

	/// attempt to open the serial device
	m_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	/// open(2) returns <0 if the port could NOT be opened
	if (m_fd == -1 ) {
		return -1;
	}
	
	/// linux serial port stuff
//	int flags;
//	flags = fcntl(m_fd,F_GETFL,0);

	fcntl(m_fd, F_SETFL, 0);// flags | O_NONBLOCK);	/// O_NONBLOCK makes read return even if there is no data

	tcgetattr(m_fd, &config);
	
	/// sets serial port baudrate
	cfsetispeed(&config, BAUD);	/// for input
	cfsetospeed(&config, BAUD);	/// for output
	
	/// adjust stop bits
	STOPBITS = StopBits;
	
	/// set data to 8-bit
	DATABITS = CS8;
	
	/// load configuration. tcsetattr(3) returns <0 if error
	//
	//config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	//Raw mode (no processing)

	config.c_cflag &= ~CRTSCTS;     //Disable hw flow ctrl  
	config.c_cflag = BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
	config.c_iflag |= INPCK | ISTRIP ;	// Enable parity checking and take away partiy bit
	config.c_iflag &= ~(IXON | IXOFF | IXANY );	//SW flow control disabled
 	config.c_oflag = 0;
 	config.c_lflag |= ICANON | ISIG ;	/// Canonical mode
	//config.c_cc[VMIN]  = 32;		
   // config.c_cc[VTIME] = 1;	//timeout after 3s without receiving new characters*/
    	/// load configuration. tcsetattr(3) returns <0 if error
    
	if(tcsetattr(m_fd, TCSANOW, &config) < 0)
	{
		return -1;
	}
	
	/// alles richtig!
	FlushInBuffer();
	FlushOutBuffer();
	PutString("E\n");
	return 0;
}

void SerialDevice::closePort()
{
	close(m_fd);
}

bool SerialDevice::checkIfStillThere()
{
	return isatty(m_fd);
}

int SerialDevice::PutString(std::string str)
{
	int res;
	
	/// write(3) returns the number of bytes that were actually written
	res = write(m_fd, str.c_str(), str.size());
	
	return res;
}

void SerialDevice::GetString( std::string& rxstr )
{
	char buf[255];
	size_t nbytes;

	nbytes = read(m_fd, buf, 255);
	//printf("Nbytes: %d", (int)nbytes);
	for(unsigned int i=0; i<nbytes; i++)
	{	
		if(buf[i] == '\n')
			break;
		rxstr.push_back(buf[i]);	
	}
}

void SerialDevice::GetStringWithEndline( std::string& rxstr )
{
        char buf[255];
        size_t nbytes;

        nbytes = read(m_fd, buf, 255);
        //printf("Nbytes: %d", (int)nbytes);
        for(unsigned int i=0; i<nbytes; i++)
        {
                //if(buf[i] == '\n')
                //      break;
                rxstr.push_back(buf[i]);
        }
}

bool SerialDevice::FlushInBuffer()
{
	if( tcflush(m_fd, TCIFLUSH) == -1 )
		return 0;
	else
		return 1;	
}


bool SerialDevice::FlushOutBuffer()
{
	if( tcflush(m_fd, TCOFLUSH) == -1 )
		return 0;
	else
		return 1;	
}

unsigned char SerialDevice::GetChar()
{
	unsigned char c;
	size_t n_bytes;
	n_bytes = read(m_fd, &c, 1);
	
	return c;
}
