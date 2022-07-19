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



}
