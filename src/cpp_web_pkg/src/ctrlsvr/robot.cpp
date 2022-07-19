#include "../stdafx.h"
#include "robot.h"
#include "controlserver.h"

using namespace std::chrono_literals;


cRobot::cRobot(const string &name)
    : Node("robot_ctrl_" + name) 
    , m_state(eState::None)
    , m_name(name)
    , m_ctrlSvr(nullptr)
    , m_rtrsPort(0)
{
}

cRobot::~cRobot()
{
    Clear();
}


// initialize robot
bool cRobot::Init(cControlServer *ctrlSvr)
{
    m_ctrlSvr = ctrlSvr;
    m_tcp.AddProtocolHandler(this);
    m_tcp.RegisterProtocol(&m_protocol);

    m_request = std::make_shared<RobotCmd::Request>();
    m_client = create_client<RobotCmd>("robot_" + m_name);
    while (!m_client->wait_for_service(1s))
        RCLCPP_INFO(get_logger(), "service not available, waiting again...");

    m_rtrs_sub = this->create_subscription<Rtrs>(
        m_name + "/rtrs", 10, std::bind(&cRobot::sub_callback, this, std::placeholders::_1));

    return true;
}


// update
bool cRobot::Update(const float deltaSeconds)
{
    m_udp.Process();

    switch (m_state)
    {
    case eState::Ready:
    {
        std::cout << "try connect cluster server "
            << m_ctrlSvr->m_clusterIp << ", " << m_ctrlSvr->m_clusterPort
            << std::endl;

        const bool result = m_ctrlSvr->m_netController.StartTcpClient(&m_tcp
            , m_ctrlSvr->m_clusterIp.c_str(), m_ctrlSvr->m_clusterPort
			, network2::DEFAULT_PACKETSIZE
			, network2::DEFAULT_PACKETCOUNT
			, network2::DEFAULT_SLEEPMILLIS);
        if (result)
        {
            m_state = eState::TryConnect;
        }
        else
        {
            std::cout << "error connect cluster server\n";

            dbg::Logc(2, "error! robot connect cluster %s %d"
                , m_ctrlSvr->m_clusterIp.c_str(), m_ctrlSvr->m_clusterPort);
            m_state = eState::Stop;
        }
    }
    break;

    case eState::TryConnect:
        if (m_tcp.IsConnect())
            m_state = eState::Connect;
        break;

    case eState::Connect:
        if (m_tcp.IsFailConnection())
        {
            m_state = eState::Stop;
            m_udp.Close();
            break;
        }
        break;
    }

    return true;
}


// connect cluster server
bool cRobot::ConnectCluster()
{
    std::cout << "connect \n";
    if ((eState::None != m_state) && (eState::Stop != m_state))
        return false;

    m_state = eState::Ready;
    return true;
}


// send request to robot service
bool cRobot::SendRequest()
{
    m_client->async_send_request(m_request
        , std::bind(&cControlServer::client_response_callback, m_ctrlSvr
                , std::placeholders::_1));
    return true;
}


// clear
void cRobot::Clear()
{
    m_ctrlSvr->m_netController.RemoveClient(&m_tcp);
    m_tcp.Close();
    m_udp.Close();
}


bool cRobot::Welcome(robot::Welcome_Packet &packet)
{
    std::cout << "receive welcome from cluster server '" << m_name << "'\n";
    m_protocol.ReqLogin(network2::SERVER_NETID, m_name);
    return true;
}


// robot protocol, ack login
bool cRobot::AckLogin(robot::AckLogin_Packet &packet)
{
    std::cout << "AckLogin " << packet.result << "\n";
    if (packet.result == 1)
    {
        m_protocol.InitializeEnd(network2::SERVER_NETID, m_name);

        m_rtrsPort = packet.rtrsSvrPort;
        m_udp.RegisterProtocol(&m_rtrsProtocol);

        std::cout << "connect udp " << m_ctrlSvr->m_clusterIp << ", " << packet.rtrsSvrPort
            << std::endl;
        m_udp.Init(m_ctrlSvr->m_clusterIp, packet.rtrsSvrPort
            , network2::DEFAULT_PACKETSIZE
		    , network2::DEFAULT_PACKETCOUNT
            , 0
            , false);

    }
    return true;
}


bool cRobot::AckLogout(robot::AckLogout_Packet &packet)
{

    return true;
}


bool cRobot::AckPathData(robot::AckPathData_Packet &packet)
{

    return true;
}


bool cRobot::AckInitializeEnd(robot::AckInitializeEnd_Packet &packet)
{

    return true;
}


bool cRobot::ReqMovable(robot::ReqMovable_Packet &packet)
{
    std::cout << "ReqMovable " << packet.name << std::endl;
    m_protocol.AckMovable(network2::SERVER_NETID, m_name, 1);
    return true;
}


bool cRobot::ReqMove(robot::ReqMove_Packet &packet)
{
    std::cout << "ReqMove " << std::endl;

    return true;
}


bool cRobot::ReqMoveCancel(robot::ReqMoveCancel_Packet &packet)
{

    return true;
}


bool cRobot::ReqWait(robot::ReqWait_Packet &packet)
{

    return true;
}


bool cRobot::ReqEmergencyStop(robot::ReqEmergencyStop_Packet &packet)
{

    return true;
}


bool cRobot::AckWork(robot::AckWork_Packet &packet)
{

    return true;
}


bool cRobot::ReqUntil(robot::ReqUntil_Packet &packet)
{

    return true;
}


bool cRobot::ReqMessage(robot::ReqMessage_Packet &packet)
{

    return true;
}

// rtrs subscriber callback
void cRobot::sub_callback(const Rtrs::SharedPtr msg)
{
    //std::cout << "recv rtrs " << m_udp.IsConnect() << std::endl;

    // RCLCPP_INFO(this->get_logger(), "I got %f", (msg->ranges)[360]);
    if (m_udp.IsConnect())
    {
        Vector3 pos(msg->posx, msg->posy, msg->posz);
        Quaternion rot(msg->orientx, msg->orienty, msg->orientz, msg->orientw);

        // ROS -> WebGL Space system
        Quaternion q;
        q.Euler(Vector3(ANGLE2RAD(-90), 0, 0));
        pos = pos * q;

        const Vector3 dir = Vector3(1,0,0) * rot * q;

        m_rtrsProtocol.RealtimeRobotState(network2::SERVER_NETID, m_name, 0, 0, 0, 0
            , pos, dir, 0);
    }
}
