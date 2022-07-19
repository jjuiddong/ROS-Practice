//------------------------------------------------------------------------
// Name:    robot_ProtocolHandler.h
// Author:  ProtocolGenerator (by jjuiddong)
// Date:    
//------------------------------------------------------------------------
#pragma once

#include "robot_ProtocolData.h"

namespace robot {

using namespace network2;
using namespace marshalling;
static const int s2c_Dispatcher_ID = 7000;

// Protocol Dispatcher
class s2c_Dispatcher: public network2::cProtocolDispatcher
{
public:
	s2c_Dispatcher();
protected:
	virtual bool Dispatch(cPacket &packet, const ProtocolHandlers &handlers) override;
	static cPacketHeader s_packetHeader;
};
static s2c_Dispatcher g_robot_s2c_Dispatcher;


// ProtocolHandler
class s2c_ProtocolHandler : virtual public network2::iProtocolHandler
{
public:
	friend class s2c_Dispatcher;
	s2c_ProtocolHandler() { m_format = ePacketFormat::BINARY; }
	virtual bool Welcome(robot::Welcome_Packet &packet) { return true; }
	virtual bool AckLogin(robot::AckLogin_Packet &packet) { return true; }
	virtual bool AckLogout(robot::AckLogout_Packet &packet) { return true; }
	virtual bool AckPathData(robot::AckPathData_Packet &packet) { return true; }
	virtual bool AckInitializeEnd(robot::AckInitializeEnd_Packet &packet) { return true; }
	virtual bool ReqMovable(robot::ReqMovable_Packet &packet) { return true; }
	virtual bool ReqMove(robot::ReqMove_Packet &packet) { return true; }
	virtual bool ReqMoveCancel(robot::ReqMoveCancel_Packet &packet) { return true; }
	virtual bool ReqWait(robot::ReqWait_Packet &packet) { return true; }
	virtual bool ReqEmergencyStop(robot::ReqEmergencyStop_Packet &packet) { return true; }
	virtual bool AckWork(robot::AckWork_Packet &packet) { return true; }
	virtual bool ReqUntil(robot::ReqUntil_Packet &packet) { return true; }
	virtual bool ReqMessage(robot::ReqMessage_Packet &packet) { return true; }
};


static const int c2s_Dispatcher_ID = 7100;

// Protocol Dispatcher
class c2s_Dispatcher: public network2::cProtocolDispatcher
{
public:
	c2s_Dispatcher();
protected:
	virtual bool Dispatch(cPacket &packet, const ProtocolHandlers &handlers) override;
	static cPacketHeader s_packetHeader;
};
static c2s_Dispatcher g_robot_c2s_Dispatcher;


// ProtocolHandler
class c2s_ProtocolHandler : virtual public network2::iProtocolHandler
{
public:
	friend class c2s_Dispatcher;
	c2s_ProtocolHandler() { m_format = ePacketFormat::BINARY; }
	virtual bool ReqLogin(robot::ReqLogin_Packet &packet) { return true; }
	virtual bool ReqLogout(robot::ReqLogout_Packet &packet) { return true; }
	virtual bool ReqPathData(robot::ReqPathData_Packet &packet) { return true; }
	virtual bool InitializeEnd(robot::InitializeEnd_Packet &packet) { return true; }
	virtual bool AckMovable(robot::AckMovable_Packet &packet) { return true; }
	virtual bool AckMove(robot::AckMove_Packet &packet) { return true; }
	virtual bool AckMoveEnd(robot::AckMoveEnd_Packet &packet) { return true; }
	virtual bool AckMoveCancel(robot::AckMoveCancel_Packet &packet) { return true; }
	virtual bool AckWait(robot::AckWait_Packet &packet) { return true; }
	virtual bool AckEmergencyStop(robot::AckEmergencyStop_Packet &packet) { return true; }
	virtual bool ReqWork(robot::ReqWork_Packet &packet) { return true; }
	virtual bool AckUntil(robot::AckUntil_Packet &packet) { return true; }
	virtual bool AwakeUntil(robot::AwakeUntil_Packet &packet) { return true; }
	virtual bool AckMessage(robot::AckMessage_Packet &packet) { return true; }
};


}
