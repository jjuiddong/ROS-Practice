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

//------------------------------------------------------------------------
// Protocol: LaserScan
//------------------------------------------------------------------------
void rtrs::c2s_Protocol::LaserScan(netid targetId, const string &robotName, const float &angle_min, const float &angle_max, const float &angle_increment, const float &time_increment, const float &scan_time, const float &range_min, const float &range_max, const vector<float> &ranges, const vector<float> &intensities)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2316814302 );
	packet << robotName;
	packet << angle_min;
	packet << angle_max;
	packet << angle_increment;
	packet << time_increment;
	packet << scan_time;
	packet << range_min;
	packet << range_max;
	packet << ranges;
	packet << intensities;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: LaserScanStream
//------------------------------------------------------------------------
void rtrs::c2s_Protocol::LaserScanStream(netid targetId, const string &robotName, const uint &total, const uint &index, const vector<float> &stream)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2623423673 );
	packet << robotName;
	packet << total;
	packet << index;
	packet << stream;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: CameraInfo
//------------------------------------------------------------------------
void rtrs::c2s_Protocol::CameraInfo(netid targetId, const string &robotName, const uint &width, const uint &height, const string &encoding)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 3375015285 );
	packet << robotName;
	packet << width;
	packet << height;
	packet << encoding;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: CameraStream
//------------------------------------------------------------------------
void rtrs::c2s_Protocol::CameraStream(netid targetId, const string &robotName, const uint &total, const uint &index, const vector<char> &stream)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1821623225 );
	packet << robotName;
	packet << total;
	packet << index;
	packet << stream;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}



