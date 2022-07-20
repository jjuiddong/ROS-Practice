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

	case 2316814302: // LaserScan
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			LaserScan_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.robotName;
			packet >> data.angle_min;
			packet >> data.angle_max;
			packet >> data.angle_increment;
			packet >> data.time_increment;
			packet >> data.scan_time;
			packet >> data.range_min;
			packet >> data.range_max;
			packet >> data.ranges;
			packet >> data.intensities;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, LaserScan(data));
		}
		break;

	case 2623423673: // LaserScanStream
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			LaserScanStream_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.robotName;
			packet >> data.total;
			packet >> data.index;
			packet >> data.stream;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, LaserScanStream(data));
		}
		break;

	case 3375015285: // CameraInfo
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			CameraInfo_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.robotName;
			packet >> data.width;
			packet >> data.height;
			packet >> data.encoding;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, CameraInfo(data));
		}
		break;

	case 1821623225: // CameraStream
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			CameraStream_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.robotName;
			packet >> data.total;
			packet >> data.index;
			packet >> data.stream;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, CameraStream(data));
		}
		break;

	default:
		assert(0);
		break;
	}
	return true;
}



