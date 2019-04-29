#include "stdafx.h"
#include "OSCSender.h"

OSCSender::OSCSender() {
	
}

void OSCSender::sendOSCMessage(std::string address, std::vector<float> &values)
{
	UdpTransmitSocket transmitSocket(IpEndpointName(ipAddress.c_str(), port));

	//Buffer
	char buffer[6144];
	osc::OutboundPacketStream p(buffer, 6144);
	p << osc::BeginBundleImmediate;
	p << osc::BeginMessage(address.c_str());
	for (int i = 0; i < values.size(); i++)
	{
		p << values[i];
	}
	p << osc::EndMessage << osc::EndBundle;
	transmitSocket.Send(p.Data(), p.Size());
}
