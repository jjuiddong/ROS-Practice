//------------------------------------------------------------------------
// Name:    rtrs_ProtocolData.h
// Author:  ProtocolGenerator (by jjuiddong)
// Date:    
//------------------------------------------------------------------------
#pragma once

namespace rtrs {

using namespace network2;
using namespace marshalling;


	struct RealtimeRobotState_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string robotName;
		int state1;
		int state2;
		int state3;
		int state4;
		Vector3 pos;
		Vector3 dir;
		double time;
	};

	struct LaserScan_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string robotName;
		float angle_min;
		float angle_max;
		float angle_increment;
		float time_increment;
		float scan_time;
		float range_min;
		float range_max;
		vector<float> ranges;
		vector<float> intensities;
	};

	struct LaserScanStream_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string robotName;
		uint total;
		uint index;
		vector<float> stream;
	};

	struct CameraInfo_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string robotName;
		uint width;
		uint height;
		string encoding;
	};

	struct CameraStream_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string robotName;
		uint total;
		uint index;
		vector<char> stream;
	};



}
