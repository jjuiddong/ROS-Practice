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
    , m_imgBuffer(nullptr)
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
        m_name + "/rtrs", 10, std::bind(&cRobot::rtrs_callback, this, std::placeholders::_1));

    m_scan_sub = this->create_subscription<LaserScan>(
        "/scan", 10, std::bind(&cRobot::scan_callback, this, std::placeholders::_1));

    m_image_sub = this->create_subscription<Image>(
        "/skidbot/camera_sensor/image_raw", 10
        , std::bind(&cRobot::image_callback, this, std::placeholders::_1));

    m_sendImageTime = now();

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
    SAFE_DELETEA(m_imgBuffer);
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
    std::cout << "robot, AckLogin " << packet.result << "\n";
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
    else
    {
        // error occurred, retry connect cluster
        m_request->type = (int)robot::eCommandType::AckLogin;
        m_request->name = m_name;
        m_request->ui0.push_back(0); // error value
        SendRequest();

        m_ctrlSvr->RetyConnectServer();
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
void cRobot::rtrs_callback(const Rtrs::SharedPtr msg)
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

void cRobot::scan_callback(const LaserScan::SharedPtr msg)
{
    //RCLCPP_INFO(get_logger(), "scan2 %d, %d", msg->ranges.size(), msg->intensities.size());

    if (!m_udp.IsConnect())
        return; // ignore

    // send scan data with streaming
    const uint byteSize = (msg->ranges.size() + msg->intensities.size()) * 4
        + (7 * 4) + 20;
    const bool isStreaming = network2::DEFAULT_PACKETSIZE <= byteSize;
    if (isStreaming)
    {
        vector<float> empty;
        m_rtrsProtocol.LaserScan(network2::SERVER_NETID, m_name, 
            msg->angle_min, msg->angle_max, msg->angle_increment
            , msg->time_increment, msg->scan_time, msg->range_min, msg->range_max
            , empty, empty);

        const uint totalBytes = (msg->ranges.size() + msg->intensities.size()) * 4;
        const uint chunkSize = network2::DEFAULT_PACKETSIZE - ((7 * 4) + 20) - 8
            - (m_name.size() + 1);
        uint totalStreamSize = totalBytes / chunkSize;
        if ((totalBytes % chunkSize) != 0)
            ++totalStreamSize;
        const uint rangesSize = msg->ranges.size();

        uint idx = 0;
        uint cursor = 0;
        while (cursor < (rangesSize + msg->intensities.size()))
        {
            vector<float> stream;
            stream.reserve(chunkSize / 4 + 1);
            uint k = 0;
            while ((k < chunkSize)
                && (cursor < (rangesSize + msg->intensities.size())))
            {
                if (cursor < rangesSize)
                    stream.push_back(msg->ranges[cursor]);
                else
                    stream.push_back(msg->intensities[cursor - rangesSize]);

                ++cursor;
                k += 4;
            }

            m_rtrsProtocol.LaserScanStream(network2::SERVER_NETID, m_name
                , totalStreamSize, idx++, stream);
        }
    }
    else
    {
        m_rtrsProtocol.LaserScan(network2::SERVER_NETID, m_name
            , msg->angle_min, msg->angle_max, msg->angle_increment
            , msg->time_increment, msg->scan_time, msg->range_min, msg->range_max
            , msg->ranges, msg->intensities);
    }

}


// camera image_raw callback
void cRobot::image_callback(const Image::SharedPtr msg)
{
    if (!m_udp.IsConnect())
        return; // ignore

    //RCLCPP_INFO(get_logger(), "send image stream %s", msg->encoding.c_str());
    if (msg->encoding != "rgb8")
        return;

    const rclcpp::Time now = this->now();
    const rclcpp::Duration duration = now - m_sendImageTime;
    if (duration.seconds() < 2.0)
        return; // ignore
    m_sendImageTime = now;

    using namespace cv;
    cv::Mat mat(800, 800, CV_8UC3, Scalar(0, 0, 0));
    // OpenCV BGR, camera image RGB
    // need converting
    //memcpy(mat.data, &msg->data[0], msg->data.size());
    for (uint i=0; i < msg->data.size(); i+=3)
    {
        mat.data[i + 0] = msg->data[i + 2]; // B
        mat.data[i + 1] = msg->data[i + 1]; // G
        mat.data[i + 2] = msg->data[i + 0]; // R
    }

    const uint newWidth = 300;
    const uint newHeight = 300;

    Mat dst;
    resize( mat, dst, cv::Size(newWidth, newHeight));

    std::vector<uchar> buff;//buffer for coding
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 80;//default(95) 0-100
    cv::imencode(".jpg", dst, buff, param);

    unsigned char *newData = &buff[0];
    const uint newDataSize = buff.size();
    //RCLCPP_INFO(get_logger(), "image compress %d", newDataSize);

    m_rtrsProtocol.CameraInfo(network2::SERVER_NETID, m_name, newWidth, newHeight, "jpeg");

    const uint totalBytes = newDataSize;
    const uint chunkSize = network2::DEFAULT_PACKETSIZE - ((2 * 4) + 20) - 8
        - (m_name.size() + 1);
    uint totalStreamSize = totalBytes / chunkSize;
    if ((totalBytes % chunkSize) != 0)
        ++totalStreamSize;

    uint idx = 0;
    uint cursor = 0;
    while (cursor < totalBytes)
    {
        const uint cpSize = min(chunkSize, totalBytes - cursor);
        vector<char> stream(cpSize);
        memcpy(&stream[0], &newData[cursor], cpSize);
        cursor += cpSize;

        //std::cout << totalBytes << ", " << totalStreamSize << ", " << idx << std::endl;
        m_rtrsProtocol.CameraStream(network2::SERVER_NETID, m_name
            , totalStreamSize, idx++, stream);
    }
}
