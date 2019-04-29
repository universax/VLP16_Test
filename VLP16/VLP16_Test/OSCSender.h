#include "Singleton.h"

#pragma once
class OSCSender :
	public Singleton<OSCSender>
{
public:
	void sendOSCMessage(std::string address, std::vector<float>& values);

private:
	friend class Singleton < OSCSender > ;
	OSCSender();
	~OSCSender(){}

	const std::string ipAddress = "127.0.0.1";
	const int port = 8000;
};

