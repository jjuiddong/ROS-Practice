
#include <iostream>
#include "stdafx.h"

#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/Net/HTTPMessage.h>
#include <Poco/Net/WebSocket.h>
#include <Poco/Net/HTTPClientSession.h>

using Poco::Net::HTTPClientSession;
using Poco::Net::HTTPRequest;
using Poco::Net::HTTPResponse;
using Poco::Net::HTTPMessage;
using Poco::Net::WebSocket;


int main() 
{
    common::cCircularQueue<char, 100> a;
    common::cCircularQueue2<char> b(200);
    common::CheckDirectoryPath("test");

    common::StrId id;
    common::CriticalSection cs;

    network2::cSession s;
    int c = 0;
    common::Vector3 v(0, 0, 0);

    using namespace network2::marshalling;
    network2::cPacket packet;
    int n = 0;
    packet >> n;

    network2::cTcpServer svr;

    network2::cWebSession ws;


    // HTTPClientSession cs("techsocietyrelay.cafe24app.com");
    // HTTPRequest request(HTTPRequest::HTTP_GET, "/?encoding=text",HTTPMessage::HTTP_1_1);
    // request.set("origin", "http://www.websocket.org");
    // HTTPResponse response;

    // try {

    //     WebSocket* m_psock = new WebSocket(cs, request, response);
    //     char const *testStr="Hello echo websocket!";
    //     char receiveBuff[256];

    //     int len=m_psock->sendFrame(testStr,strlen(testStr),WebSocket::FRAME_TEXT);
    //     std::cout << "Sent bytes " << len << std::endl;
    //     int flags=0;

    //     int rlen=m_psock->receiveFrame(receiveBuff,256,flags);
    //     std::cout << "Received bytes " << rlen << std::endl;
    //     std::cout << receiveBuff << std::endl;

    //     m_psock->close();

    // } catch (std::exception &e) {
    //     std::cout << "Exception " << e.what();
    // }

    return 1;
}
