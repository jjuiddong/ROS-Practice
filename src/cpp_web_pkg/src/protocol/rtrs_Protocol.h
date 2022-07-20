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
	void LaserScan(netid targetId, const string &robotName, const float &angle_min, const float &angle_max, const float &angle_increment, const float &time_increment, const float &scan_time, const float &range_min, const float &range_max, const vector<float> &ranges, const vector<float> &intensities);
	void LaserScanStream(netid targetId, const string &robotName, const uint &total, const uint &index, const vector<float> &stream);
	void CameraInfo(netid targetId, const string &robotName, const uint &width, const uint &height, const string &encoding);
	void CameraStream(netid targetId, const string &robotName, const uint &total, const uint &index, const vector<char> &stream);
	static cPacketHeader s_packetHeader;
};
}
