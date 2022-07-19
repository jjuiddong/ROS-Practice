#include "../stdafx.h"
#include "rtrs_Protocol.h"
using namespace rtrs;

cPacketHeader rtrs::c2s_Protocol::s_packetHeader;
//------------------------------------------------------------------------
// Protocol: RealtimeRobotState
//------------------------------------------------------------------------
void rtrs::c2s_Protocol::RealtimeRobotState(netid targetId, const string &robotName, const int &state1, const int &state2, const int &state3, const int &state4, const Vector3 &pos, const Vector3 &dir, const double &time)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1079519326 );
	packet << robotName;
	packet << state1;
	packet << state2;
	packet << state3;
	packet << state4;
	packet << pos;
	packet << dir;
	packet << time;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}



