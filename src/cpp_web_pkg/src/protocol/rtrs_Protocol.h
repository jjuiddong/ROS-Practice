//------------------------------------------------------------------------
// Name:    rtrs_Protocol.h
// Author:  ProtocolGenerator (by jjuiddong)
// Date:    
//------------------------------------------------------------------------
#pragma once

namespace rtrs {

using namespace network2;
using namespace marshalling;
static const int c2s_Protocol_ID = 7101;

class c2s_Protocol : public network2::iProtocol
{
public:
	c2s_Protocol() : iProtocol(c2s_Protocol_ID, ePacketFormat::BINARY) {}
	void RealtimeRobotState(netid targetId, const string &robotName, const int &state1, const int &state2, const int &state3, const int &state4, const Vector3 &pos, const Vector3 &dir, const double &time);
	static cPacketHeader s_packetHeader;
};
}
