#include "stdafx.h"
#include "OSCSender.h"

OSCSender::OSCSender() {
	
}

void OSCSender::sendOSCMessage(int numObject, int index, float x, float y, float z)
{
	UdpTransmitSocket transmitSocket(IpEndpointName(ipAddress.c_str(), port));

	//Buffer
	char buffer[6144];
	osc::OutboundPacketStream p(buffer, 6144);
	p << osc::BeginBundleImmediate
		//Head
		<< osc::BeginMessage("/position") << numObject << index << x << y << z << osc::EndMessage
		<< osc::EndBundle;
	transmitSocket.Send(p.Data(), p.Size());
}
