#include "../stdafx.h"
#include "robot_Protocol.h"
using namespace robot;

cPacketHeader robot::s2c_Protocol::s_packetHeader;
//------------------------------------------------------------------------
// Protocol: Welcome
//------------------------------------------------------------------------
void robot::s2c_Protocol::Welcome(netid targetId, const string &msg)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1281093745 );
	packet << msg;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckLogin
//------------------------------------------------------------------------
void robot::s2c_Protocol::AckLogin(netid targetId, const string &name, const int &rtrsSvrPort, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 851424104 );
	packet << name;
	packet << rtrsSvrPort;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckLogout
//------------------------------------------------------------------------
void robot::s2c_Protocol::AckLogout(netid targetId, const string &name, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2822050476 );
	packet << name;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckPathData
//------------------------------------------------------------------------
void robot::s2c_Protocol::AckPathData(netid targetId, const BYTE &id, const BYTE &result, const BYTE &count, const BYTE &index, const uint &bufferSize, const vector<BYTE> &data)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2958091647 );
	packet << id;
	packet << result;
	packet << count;
	packet << index;
	packet << bufferSize;
	packet << data;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckInitializeEnd
//------------------------------------------------------------------------
void robot::s2c_Protocol::AckInitializeEnd(netid targetId)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1960946453 );
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqMovable
//------------------------------------------------------------------------
void robot::s2c_Protocol::ReqMovable(netid targetId, const string &name, const vector<ushort> &path)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2020016972 );
	packet << name;
	packet << path;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqMove
//------------------------------------------------------------------------
void robot::s2c_Protocol::ReqMove(netid targetId, const string &name, const vector<ushort> &path, const vector<ushort> &wtimes)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1435916264 );
	packet << name;
	packet << path;
	packet << wtimes;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqMoveCancel
//------------------------------------------------------------------------
void robot::s2c_Protocol::ReqMoveCancel(netid targetId, const string &name)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1660844561 );
	packet << name;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqWait
//------------------------------------------------------------------------
void robot::s2c_Protocol::ReqWait(netid targetId, const string &name, const int &delayTime)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 3400304571 );
	packet << name;
	packet << delayTime;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqEmergencyStop
//------------------------------------------------------------------------
void robot::s2c_Protocol::ReqEmergencyStop(netid targetId, const string &name, const int &type)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 3867147673 );
	packet << name;
	packet << type;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckWork
//------------------------------------------------------------------------
void robot::s2c_Protocol::AckWork(netid targetId, const string &name, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 3785607084 );
	packet << name;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqUntil
//------------------------------------------------------------------------
void robot::s2c_Protocol::ReqUntil(netid targetId, const string &name, const int &vmId, const string &smsg, const string &fmsg, const int &time)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 3387050074 );
	packet << name;
	packet << vmId;
	packet << smsg;
	packet << fmsg;
	packet << time;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqMessage
//------------------------------------------------------------------------
void robot::s2c_Protocol::ReqMessage(netid targetId, const string &name, const int &vmId, const string &sender, const string &msg)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 656884565 );
	packet << name;
	packet << vmId;
	packet << sender;
	packet << msg;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}



cPacketHeader robot::c2s_Protocol::s_packetHeader;
//------------------------------------------------------------------------
// Protocol: ReqLogin
//------------------------------------------------------------------------
void robot::c2s_Protocol::ReqLogin(netid targetId, const string &name)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1956887904 );
	packet << name;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqLogout
//------------------------------------------------------------------------
void robot::c2s_Protocol::ReqLogout(netid targetId, const string &name)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1095604361 );
	packet << name;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqPathData
//------------------------------------------------------------------------
void robot::c2s_Protocol::ReqPathData(netid targetId, const string &name)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 3691343431 );
	packet << name;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: InitializeEnd
//------------------------------------------------------------------------
void robot::c2s_Protocol::InitializeEnd(netid targetId, const string &name)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1758265624 );
	packet << name;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckMovable
//------------------------------------------------------------------------
void robot::c2s_Protocol::AckMovable(netid targetId, const string &name, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 4250648589 );
	packet << name;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckMove
//------------------------------------------------------------------------
void robot::c2s_Protocol::AckMove(netid targetId, const string &name, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 932256373 );
	packet << name;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckMoveEnd
//------------------------------------------------------------------------
void robot::c2s_Protocol::AckMoveEnd(netid targetId, const string &name, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2005804628 );
	packet << name;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckMoveCancel
//------------------------------------------------------------------------
void robot::c2s_Protocol::AckMoveCancel(netid targetId, const string &name, const int &stopVtxIdx, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 1050611930 );
	packet << name;
	packet << stopVtxIdx;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckWait
//------------------------------------------------------------------------
void robot::c2s_Protocol::AckWait(netid targetId, const string &name, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2998388166 );
	packet << name;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckEmergencyStop
//------------------------------------------------------------------------
void robot::c2s_Protocol::AckEmergencyStop(netid targetId, const string &name, const int &type, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 3902930067 );
	packet << name;
	packet << type;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: ReqWork
//------------------------------------------------------------------------
void robot::c2s_Protocol::ReqWork(netid targetId, const string &name)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 3933130150 );
	packet << name;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckUntil
//------------------------------------------------------------------------
void robot::c2s_Protocol::AckUntil(netid targetId, const string &name, const int &vmId, const string &smsg, const string &fmsg, const int &time, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2582431902 );
	packet << name;
	packet << vmId;
	packet << smsg;
	packet << fmsg;
	packet << time;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AwakeUntil
//------------------------------------------------------------------------
void robot::c2s_Protocol::AwakeUntil(netid targetId, const string &name, const int &state, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 2775462807 );
	packet << name;
	packet << state;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}

//------------------------------------------------------------------------
// Protocol: AckMessage
//------------------------------------------------------------------------
void robot::c2s_Protocol::AckMessage(netid targetId, const string &name, const int &vmId, const string &sender, const string &msg, const int &result)
{
	cPacket packet(&s_packetHeader);
	packet.SetProtocolId( GetId() );
	packet.SetPacketId( 328063734 );
	packet << name;
	packet << vmId;
	packet << sender;
	packet << msg;
	packet << result;
	packet.EndPack();
	GetNode()->Send(targetId, packet);
}



