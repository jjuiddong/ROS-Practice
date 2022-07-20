//------------------------------------------------------------------------
// Name:    rtrs_ProtocolHandler.h
// Author:  ProtocolGenerator (by jjuiddong)
// Date:    
//------------------------------------------------------------------------
#pragma once

#include "rtrs_ProtocolData.h"

namespace rtrs {

using namespace network2;
using namespace marshalling;
static const int c2s_Dispatcher_ID = 7101;

// Protocol Dispatcher
class c2s_Dispatcher: public network2::cProtocolDispatcher
{
public:
	c2s_Dispatcher();
protected:
	virtual bool Dispatch(cPacket &packet, const ProtocolHandlers &handlers) override;
	static cPacketHeader s_packetHeader;
};
static c2s_Dispatcher g_rtrs_c2s_Dispatcher;


// ProtocolHandler
class c2s_ProtocolHandler : virtual public network2::iProtocolHandler
{
public:
	friend class c2s_Dispatcher;
	c2s_ProtocolHandler() { m_format = ePacketFormat::BINARY; }
	virtual bool RealtimeRobotState(rtrs::RealtimeRobotState_Packet &packet) { return true; }
	virtual bool LaserScan(rtrs::LaserScan_Packet &packet) { return true; }
	virtual bool LaserScanStream(rtrs::LaserScanStream_Packet &packet) { return true; }
	virtual bool CameraInfo(rtrs::CameraInfo_Packet &packet) { return true; }
	virtual bool CameraStream(rtrs::CameraStream_Packet &packet) { return true; }
};


}
