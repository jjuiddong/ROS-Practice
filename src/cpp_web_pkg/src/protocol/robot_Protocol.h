//------------------------------------------------------------------------
// Name:    robot_Protocol.h
// Author:  ProtocolGenerator (by jjuiddong)
// Date:    
//------------------------------------------------------------------------
#pragma once

namespace robot {

using namespace network2;
using namespace marshalling;
static const int s2c_Protocol_ID = 7000;

class s2c_Protocol : public network2::iProtocol
{
public:
	s2c_Protocol() : iProtocol(s2c_Protocol_ID, ePacketFormat::BINARY) {}
	void Welcome(netid targetId, const string &msg);
	void AckLogin(netid targetId, const string &name, const int &rtrsSvrPort, const int &result);
	void AckLogout(netid targetId, const string &name, const int &result);
	void AckPathData(netid targetId, const BYTE &id, const BYTE &result, const BYTE &count, const BYTE &index, const uint &bufferSize, const vector<BYTE> &data);
	void AckInitializeEnd(netid targetId);
	void ReqMovable(netid targetId, const string &name, const vector<ushort> &path);
	void ReqMove(netid targetId, const string &name, const vector<ushort> &path, const vector<ushort> &wtimes);
	void ReqMoveCancel(netid targetId, const string &name);
	void ReqWait(netid targetId, const string &name, const int &delayTime);
	void ReqEmergencyStop(netid targetId, const string &name, const int &type);
	void AckWork(netid targetId, const string &name, const int &result);
	void ReqUntil(netid targetId, const string &name, const int &vmId, const string &smsg, const string &fmsg, const int &time);
	void ReqMessage(netid targetId, const string &name, const int &vmId, const string &sender, const string &msg);
	static cPacketHeader s_packetHeader;
};
static const int c2s_Protocol_ID = 7100;

class c2s_Protocol : public network2::iProtocol
{
public:
	c2s_Protocol() : iProtocol(c2s_Protocol_ID, ePacketFormat::BINARY) {}
	void ReqLogin(netid targetId, const string &name);
	void ReqLogout(netid targetId, const string &name);
	void ReqPathData(netid targetId, const string &name);
	void InitializeEnd(netid targetId, const string &name);
	void AckMovable(netid targetId, const string &name, const int &result);
	void AckMove(netid targetId, const string &name, const int &result);
	void AckMoveEnd(netid targetId, const string &name, const int &result);
	void AckMoveCancel(netid targetId, const string &name, const int &stopVtxIdx, const int &result);
	void AckWait(netid targetId, const string &name, const int &result);
	void AckEmergencyStop(netid targetId, const string &name, const int &type, const int &result);
	void ReqWork(netid targetId, const string &name);
	void AckUntil(netid targetId, const string &name, const int &vmId, const string &smsg, const string &fmsg, const int &time, const int &result);
	void AwakeUntil(netid targetId, const string &name, const int &state, const int &result);
	void AckMessage(netid targetId, const string &name, const int &vmId, const string &sender, const string &msg, const int &result);
	static cPacketHeader s_packetHeader;
};
}
