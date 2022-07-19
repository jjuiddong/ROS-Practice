//------------------------------------------------------------------------
// Name:    robot_ProtocolData.h
// Author:  ProtocolGenerator (by jjuiddong)
// Date:    
//------------------------------------------------------------------------
#pragma once

namespace robot {

using namespace network2;
using namespace marshalling;


	struct Welcome_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string msg;
	};

	struct AckLogin_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int rtrsSvrPort;
		int result;
	};

	struct AckLogout_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int result;
	};

	struct AckPathData_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		BYTE id;
		BYTE result;
		BYTE count;
		BYTE index;
		uint bufferSize;
		vector<BYTE> data;
	};

	struct AckInitializeEnd_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
	};

	struct ReqMovable_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		vector<ushort> path;
	};

	struct ReqMove_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		vector<ushort> path;
		vector<ushort> wtimes;
	};

	struct ReqMoveCancel_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
	};

	struct ReqWait_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int delayTime;
	};

	struct ReqEmergencyStop_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int type;
	};

	struct AckWork_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int result;
	};

	struct ReqUntil_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int vmId;
		string smsg;
		string fmsg;
		int time;
	};

	struct ReqMessage_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int vmId;
		string sender;
		string msg;
	};





	struct ReqLogin_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
	};

	struct ReqLogout_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
	};

	struct ReqPathData_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
	};

	struct InitializeEnd_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
	};

	struct AckMovable_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int result;
	};

	struct AckMove_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int result;
	};

	struct AckMoveEnd_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int result;
	};

	struct AckMoveCancel_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int stopVtxIdx;
		int result;
	};

	struct AckWait_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int result;
	};

	struct AckEmergencyStop_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int type;
		int result;
	};

	struct ReqWork_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
	};

	struct AckUntil_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int vmId;
		string smsg;
		string fmsg;
		int time;
		int result;
	};

	struct AwakeUntil_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int state;
		int result;
	};

	struct AckMessage_Packet {
		cProtocolDispatcher *pdispatcher;
		netid senderId;
		string name;
		int vmId;
		string sender;
		string msg;
		int result;
	};



}
