#ifndef CHAI3D_SIMPLE_SERIAL_H
#define CHAI3D_SIMPLE_SERIAL_H

#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <string.h>
#include <chrono>
#include <thread>
#include <time.h>
#include <fstream>

namespace chai3d {

	class SimpleSerial
	{

	private:
		HANDLE io_handler_;
		COMSTAT status_;
		DWORD errors_;

		std::string syntax_name_;
		char front_delimiter_;
		char end_delimiter_;

		void CustomSyntax(std::string syntax_type);

	public:
		SimpleSerial(char* com_port, DWORD COM_BAUD_RATE);

		std::string ReadSerialPort(int reply_wait_time);
		bool WriteSerialPort(char* data_sent);
		bool CloseSerialPort();
		~SimpleSerial();
		bool connected_;
	};

}

#endif