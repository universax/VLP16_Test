#include "stdafx.h"

#pragma once
class OSCReceiveHandler : public osc::OscPacketListener {
private:

public:
	OSCReceiveHandler(){}
	~OSCReceiveHandler(){}
protected:

	bool isOSCReceiveBegan = false;

	virtual void ProcessMessage(const osc::ReceivedMessage& m,
		const IpEndpointName& remoteEndpoint)
	{
		(void)remoteEndpoint; // suppress unused parameter warning

		try{
			// example of parsing single messages. osc::OsckPacketListener
			// handles the bundle traversal.
			if (strcmp(m.AddressPattern(), "/address") == 0){
				// example #1 -- argument stream interface
				
			}
		}
		catch (osc::Exception& e){
			// any parsing errors such as unexpected argument types, or 
			// missing arguments get thrown as exceptions.
			std::cout << "OSC Receive: Error while parsing message: "
				<< m.AddressPattern() << ": " << e.what() << "\n";
		}
	}
};

