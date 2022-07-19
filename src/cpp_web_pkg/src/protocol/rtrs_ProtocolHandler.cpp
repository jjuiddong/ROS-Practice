#include "../stdafx.h"
#include "rtrs_ProtocolHandler.h"

using namespace rtrs;


cPacketHeader rtrs::c2s_Dispatcher::s_packetHeader;
rtrs::c2s_Dispatcher::c2s_Dispatcher()
	: cProtocolDispatcher(rtrs::c2s_Dispatcher_ID, ePacketFormat::BINARY)
{
	cProtocolDispatcher::GetDispatcherMap()->insert({c2s_Dispatcher_ID, this});
	cProtocolDispatcher::GetPacketHeaderMap()->insert({c2s_Dispatcher_ID, &s_packetHeader});
}

//------------------------------------------------------------------------
// ��Ŷ�� �������ݿ� ���� �ش��ϴ� �ڵ鷯�� ȣ���Ѵ�.
//------------------------------------------------------------------------
bool rtrs::c2s_Dispatcher::Dispatch(cPacket &packet, const ProtocolHandlers &handlers)
{
	const int protocolId = packet.GetProtocolId();
	const int packetId = packet.GetPacketId();
	switch (packetId)
	{
	case 1079519326: // RealtimeRobotState
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			RealtimeRobotState_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.robotName;
			packet >> data.state1;
			packet >> data.state2;
			packet >> data.state3;
			packet >> data.state4;
			packet >> data.pos;
			packet >> data.dir;
			packet >> data.time;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, RealtimeRobotState(data));
		}
		break;

	default:
		assert(0);
		break;
	}
	return true;
}



