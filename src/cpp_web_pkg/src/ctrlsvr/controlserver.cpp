
#include "../stdafx.h"
#include "controlserver.h"
#include "robot.h"

using namespace std;


cControlServer::cControlServer()
    : Node("robot_ctrl_server") 
    , m_state(eState::None)
    , m_reconTime(0.f)
    , m_connTime(0.f)
{
}

cControlServer::~cControlServer()
{
    Clear();
}


// initialize control server
bool cControlServer::Init(const sInitParam &param)
{
    m_param = param;
    m_client.RegisterProtocol(&m_protocol);
    m_client.AddProtocolHandler(this);

    m_state = eState::TryConnect;
    m_relaySvrUrl = param.url;
    m_connTime = 0.f;

    cout << "start ros control server \n";
    cout << "connect to relaysvr = " << param.url << "\n";

    if (!m_netController.StartWebClient(&m_client, param.url
        , network2::DEFAULT_PACKETSIZE, network2::DEFAULT_PACKETCOUNT
        , network2::DEFAULT_SLEEPMILLIS, true))
    {
        std::cout << "webclient initialize error \n";

        return false;
    }

    m_service = create_service<RobotCmd>("robotserver"
        , std::bind(&cControlServer::response_callback, this
            , std::placeholders::_1, std::placeholders::_2));

    return true;
}


// process control server
bool cControlServer::Update(const float deltaSeconds)
{
    if (eState::Stop == m_state)
        return false;

    m_netController.Process(1);

    for (auto &r : m_robots)
    {
        r->Update(deltaSeconds);
        rclcpp::spin_some(r);
    }

    switch (m_state)
    {
    case eState::TryConnect:
        m_connTime += deltaSeconds;
        if (m_connTime > 5.f)
        {
            cout << "fail connection, time over\n";
            m_state = eState::WaitCluster; // retry later
            m_client.Close();
        }
        else if (m_client.IsFailConnection())
        {
            cout << "fail connection\n";
            m_state = eState::WaitCluster; // retry later
        }
        break;
    
    case eState::SyncCluster:
        for (auto &robot : m_robots)
        {
            if (cRobot::eState::Stop == robot->m_state)
            {
                // disconnect to robot control server?
                // retry receive cluster ip/port
                m_state = eState::WaitCluster;
                cout << "close when connect cluster\n";
            }
        }
        break;

    case eState::WaitCluster:
        m_reconTime += deltaSeconds;
        if (m_reconTime > 3.f)
        {
            ReConnect();
            m_reconTime = 0.f;
        }
        break;

    default:
        break;
    }

    // robot task process
    if (!m_tasks.empty())
    {
        sTask task = m_tasks.front();
        m_tasks.pop();

        cRobot *robot = FindSession(task.robotName);
        if (robot)
        {
            switch (task.type)
            {
            case robot::eCommandType::EmptyWork:
            {
                // robot->m_request->type = (int)robot::eCommandType::ReqRemoteMove;
                // robot->m_request->name = task.robotName;
                // robot->m_request->f0.clear();
                // robot->m_request->f0.push_back(5);
                // robot->m_request->f0.push_back(1.f);
                // robot->m_request->f0.push_back(5.f);
                // robot->SendRequest();
            }
            break;
            }
        }
    }

    return true;
}


// retry connect web relay server
void cControlServer::RetyConnectServer()
{
    m_state = eState::WaitCluster; // retry later
    m_client.Close();
    m_robots.clear();
}


void cControlServer::Clear()
{
    m_client.Close();
    m_netController.Clear();
    m_state = eState::Stop;

    m_robots.clear();
}


// find robot session form robot name
cRobot* cControlServer::FindSession(const string &robotName)
{
    for (auto &robot : m_robots)
    {
        if (robot->m_name == robotName)
            return robot.get();
    }
    return nullptr;
}


// reconnect relay server for check cluster alive
bool cControlServer::ReConnect()
{
    cout << "reconnect relay server \n";

    m_connTime = 0.f;

    if (!m_netController.StartWebClient(&m_client, m_relaySvrUrl
        , network2::DEFAULT_PACKETSIZE, network2::DEFAULT_PACKETCOUNT
        , network2::DEFAULT_SLEEPMILLIS, true))
    {
        std::cout << "webclient initialize error \n";
        m_state = eState::WaitCluster; // retry next
        return false;
    }

    m_state = eState::TryConnect;
    return true;
}


// first connection packet 
bool cControlServer::Welcome(webrelay::Welcome_Packet &packet)
{
    if (eState::TryConnect != m_state)
        return true;

    std::cout << "Success Connect RelayServer, msg = " << packet.msg << "\n";
    // 7: Ros Control Server type
    m_protocol.ReqLogin(network2::SERVER_NETID, true, "ros-jjuiddong", 7); 
    return true;
}


// response login req
bool cControlServer::AckLogin(webrelay::AckLogin_Packet &packet)
{
    std::cout << "AckLogin " << packet.result << std::endl;
    if (1 != packet.result)
    {
        // error occurred, reconnect web relay server
        std::cout << "error occurred, reconnect web relay server" << std::endl;
        m_state = eState::WaitCluster; // retry later
        m_client.Close();
    }

    return true;
}


// reesponse logout req
bool cControlServer::AckLogout(webrelay::AckLogout_Packet &packet)
{
    return true;
}


// request connect cluster server
bool cControlServer::ReqConnectProxyServer(webrelay::ReqConnectProxyServer_Packet &packet)
{
    if (eState::TryConnect != m_state)
        return true;

    int result = packet.url.empty()? 0 : 1;
    std::cout << "ReqConnectProxyServer result = " << result 
        << ", url = " << packet.url << std::endl;
    if (packet.url.empty())
    {
        m_state = eState::WaitCluster; // retry next
        m_client.Close();
        return true; // fail
    }

    // packet.url format = 127.0.0.1:9999
    vector<string> toks;
    common::tokenizer(packet.url, ":", "", toks);
    if (toks.size() == 2)
    {
        m_clusterIp = toks[0];
        m_clusterPort = atoi(toks[1].c_str());
        m_state = eState::SyncCluster; // success receive cluster ip

        for (auto &robot : m_robots)
            robot->ConnectCluster();
    }
    else 
    {
        m_state = eState::WaitCluster; // retry next
    }

    m_client.Close();
    return true;
}


// service RobotCmd packet response
void cControlServer::response_callback(std::shared_ptr<RobotCmd::Request> request,
                        std::shared_ptr<RobotCmd::Response> response) 
{
    RCLCPP_INFO(get_logger(), "Receive RobotCmd %d", request->type);
    
    const robot::eCommandType type = (robot::eCommandType)request->type;
    switch (type)
    {
    case robot::eCommandType::ReqLogin:
    {
        response->type = (int)robot::eCommandType::AckLogin;
        response->result = 1;

        auto robot = std::make_shared<cRobot>(request.get()->name);
        robot->Init(this);
        m_robots.push_back(robot);

        std::cout << "ip = " << m_clusterIp << std::endl;
        if (eState::SyncCluster == m_state)
            robot->ConnectCluster();
    }
    break;

    case robot::eCommandType::EmptyWork:
    {
        response->type = (int)robot::eCommandType::AckEmptyWork;

        cRobot *robot = FindSession(request.get()->name);
        if (!robot)
        {
            response->result = 0; // not found robot
            return;
        }

        response->result = 1;

        if ((eState::SyncCluster == m_state) && (cRobot::eState::Connect == robot->m_state))
        {
            sTask task;
            task.robotName = robot->m_name;
            task.type = type;
            m_tasks.push(task);
        }
        else
        {
            response->result = 0; // disconnect
        }
    }
    break;
    }
}


// robot client response callback
void cControlServer::client_response_callback(SharedFuture result)
{
    RCLCPP_INFO(get_logger(), "Receive RobotClient Response RobotCmd %d", result.get()->type);

    const robot::eCommandType type = (robot::eCommandType)result.get()->type;
    switch (type)
    {
    case robot::eCommandType::AckRemoteMove:
        RCLCPP_INFO(get_logger(), "Ack Remote Move result = %d", result.get()->result);
        break;
    case robot::eCommandType::AckRemoteMoveEnd:
        RCLCPP_INFO(get_logger(), "Ack Remote Move End result = %d", result.get()->result);
        break;

    default:
        RCLCPP_INFO(get_logger(), "unknown response from robot client");
        break;
    }
}
