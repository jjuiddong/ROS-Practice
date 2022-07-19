#include "../stdafx.h"
#include "robot_ProtocolHandler.h"

using namespace robot;


cPacketHeader robot::s2c_Dispatcher::s_packetHeader;
robot::s2c_Dispatcher::s2c_Dispatcher()
	: cProtocolDispatcher(robot::s2c_Dispatcher_ID, ePacketFormat::BINARY)
{
	cProtocolDispatcher::GetDispatcherMap()->insert({s2c_Dispatcher_ID, this});
	cProtocolDispatcher::GetPacketHeaderMap()->insert({s2c_Dispatcher_ID, &s_packetHeader});
}

//------------------------------------------------------------------------
// ��Ŷ�� �������ݿ� ���� �ش��ϴ� �ڵ鷯�� ȣ���Ѵ�.
//------------------------------------------------------------------------
bool robot::s2c_Dispatcher::Dispatch(cPacket &packet, const ProtocolHandlers &handlers)
{
	const int protocolId = packet.GetProtocolId();
	const int packetId = packet.GetPacketId();
	switch (packetId)
	{
	case 1281093745: // Welcome
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			Welcome_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.msg;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, Welcome(data));
		}
		break;

	case 851424104: // AckLogin
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckLogin_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.rtrsSvrPort;
			packet >> data.result;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, AckLogin(data));
		}
		break;

	case 2822050476: // AckLogout
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckLogout_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.result;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, AckLogout(data));
		}
		break;

	case 2958091647: // AckPathData
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckPathData_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.id;
			packet >> data.result;
			packet >> data.count;
			packet >> data.index;
			packet >> data.bufferSize;
			packet >> data.data;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, AckPathData(data));
		}
		break;

	case 1960946453: // AckInitializeEnd
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckInitializeEnd_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, AckInitializeEnd(data));
		}
		break;

	case 2020016972: // ReqMovable
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqMovable_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.path;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, ReqMovable(data));
		}
		break;

	case 1435916264: // ReqMove
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqMove_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.path;
			packet >> data.wtimes;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, ReqMove(data));
		}
		break;

	case 1660844561: // ReqMoveCancel
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqMoveCancel_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, ReqMoveCancel(data));
		}
		break;

	case 3400304571: // ReqWait
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqWait_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.delayTime;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, ReqWait(data));
		}
		break;

	case 3867147673: // ReqEmergencyStop
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqEmergencyStop_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.type;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, ReqEmergencyStop(data));
		}
		break;

	case 3785607084: // AckWork
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckWork_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.result;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, AckWork(data));
		}
		break;

	case 3387050074: // ReqUntil
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqUntil_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.vmId;
			packet >> data.smsg;
			packet >> data.fmsg;
			packet >> data.time;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, ReqUntil(data));
		}
		break;

	case 656884565: // ReqMessage
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<s2c_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqMessage_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.vmId;
			packet >> data.sender;
			packet >> data.msg;
			SEND_HANDLER(s2c_ProtocolHandler, prtHandler, ReqMessage(data));
		}
		break;

	default:
		assert(0);
		break;
	}
	return true;
}




cPacketHeader robot::c2s_Dispatcher::s_packetHeader;
robot::c2s_Dispatcher::c2s_Dispatcher()
	: cProtocolDispatcher(robot::c2s_Dispatcher_ID, ePacketFormat::BINARY)
{
	cProtocolDispatcher::GetDispatcherMap()->insert({c2s_Dispatcher_ID, this});
	cProtocolDispatcher::GetPacketHeaderMap()->insert({c2s_Dispatcher_ID, &s_packetHeader});
}

//------------------------------------------------------------------------
// ��Ŷ�� �������ݿ� ���� �ش��ϴ� �ڵ鷯�� ȣ���Ѵ�.
//------------------------------------------------------------------------
bool robot::c2s_Dispatcher::Dispatch(cPacket &packet, const ProtocolHandlers &handlers)
{
	const int protocolId = packet.GetProtocolId();
	const int packetId = packet.GetPacketId();
	switch (packetId)
	{
	case 1956887904: // ReqLogin
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqLogin_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, ReqLogin(data));
		}
		break;

	case 1095604361: // ReqLogout
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqLogout_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, ReqLogout(data));
		}
		break;

	case 3691343431: // ReqPathData
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqPathData_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, ReqPathData(data));
		}
		break;

	case 1758265624: // InitializeEnd
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			InitializeEnd_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, InitializeEnd(data));
		}
		break;

	case 4250648589: // AckMovable
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckMovable_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AckMovable(data));
		}
		break;

	case 932256373: // AckMove
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckMove_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AckMove(data));
		}
		break;

	case 2005804628: // AckMoveEnd
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckMoveEnd_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AckMoveEnd(data));
		}
		break;

	case 1050611930: // AckMoveCancel
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckMoveCancel_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.stopVtxIdx;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AckMoveCancel(data));
		}
		break;

	case 2998388166: // AckWait
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckWait_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AckWait(data));
		}
		break;

	case 3902930067: // AckEmergencyStop
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckEmergencyStop_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.type;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AckEmergencyStop(data));
		}
		break;

	case 3933130150: // ReqWork
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			ReqWork_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, ReqWork(data));
		}
		break;

	case 2582431902: // AckUntil
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckUntil_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.vmId;
			packet >> data.smsg;
			packet >> data.fmsg;
			packet >> data.time;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AckUntil(data));
		}
		break;

	case 2775462807: // AwakeUntil
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AwakeUntil_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.state;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AwakeUntil(data));
		}
		break;

	case 328063734: // AckMessage
		{
			ProtocolHandlers prtHandler;
			if (!HandlerMatching<c2s_ProtocolHandler>(handlers, prtHandler))
				return false;

			SetCurrentDispatchPacket( &packet );

			AckMessage_Packet data;
			data.pdispatcher = this;
			data.senderId = packet.GetSenderId();
			packet >> data.name;
			packet >> data.vmId;
			packet >> data.sender;
			packet >> data.msg;
			packet >> data.result;
			SEND_HANDLER(c2s_ProtocolHandler, prtHandler, AckMessage(data));
		}
		break;

	default:
		assert(0);
		break;
	}
	return true;
}



