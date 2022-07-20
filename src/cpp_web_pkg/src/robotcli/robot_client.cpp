//
// 2022-06-04, jjuiddong
//  - robot client
//    - connect RobotCmd Service
//    - 1. Login
//    - 2. Request Task
//
#include "../stdafx.h"
#include "robot_client.h"

using namespace std::chrono_literals;


cRobotClient::cRobotClient(const string &name) 
    : Node("robot_client")
    , m_state(eState::None)
    , m_name(name)
    , m_emptyWorkTime(0.f)
    , m_timeDuration(0.f)
    , m_linearVelX(0.f)
    , m_angularVelZ(0.f)
    , m_rtrsTime(0.f)
{
}

cRobotClient::~cRobotClient()
{
    m_state = eState::Stop;
}


// initialize robot
bool cRobotClient::Init()
{
    m_state = eState::Initialize;
    m_service = create_service<RobotCmd>("robot_" + m_name
        , std::bind(&cRobotClient::server_response_callback, this
            , std::placeholders::_1, std::placeholders::_2));
    m_client = create_client<RobotCmd>("robotserver");
    m_request = std::make_shared<RobotCmd::Request>();
    m_twist_pub = create_publisher<Twist>("skidbot/cmd_vel", 10);
    m_rtrs_pub = create_publisher<Rtrs>(m_name + "/rtrs", 10);
    m_pose_sub = create_subscription<Odom>(
        "/odom", 10, std::bind(&cRobotClient::odom_callback, this, std::placeholders::_1));

    m_rtrs_msg.name = m_name;
    m_rtrs_msg.posx = 0.f;
    m_rtrs_msg.posy = 0.f;
    m_rtrs_msg.posz = 0.f;
    m_rtrs_msg.orientx = 0.f;
    m_rtrs_msg.orienty = 0.f;
    m_rtrs_msg.orientz = 0.f;
    m_rtrs_msg.orientw = 0.f;
    m_rtrs_msg.time = 0.0;

    RCLCPP_INFO(get_logger(), "service not available, waiting again...");    

    m_state = eState::ConnectService;
    return true;
}


// update
bool cRobotClient::Update(const float deltaSeconds)
{
    switch (m_state)
    {
    case eState::ConnectService:
        if (m_client->wait_for_service(1s))
        {
            RCLCPP_INFO(get_logger(), "service available, waiting serice call");            
            m_state = eState::Login;
            ReqLogin();
        }
        else
        {
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");            
        }
        break;

    case eState::Login:
        break;
    case eState::Ready:
    {
        m_emptyWorkTime += deltaSeconds;
        if (m_emptyWorkTime > 3.f)
        {
            m_emptyWorkTime = 0.f;
            EmptyWork();
        }
    }
    break;

    case eState::Move:
    {
        m_timeDuration -= deltaSeconds;
        MoveRobot(m_linearVelX, m_angularVelZ);
        if (m_timeDuration < 0.f)
        {
            StopRobot();
            m_emptyWorkTime = 0.f;
            m_state = eState::Ready;
        }
    }
    break;
    
    default: break;
    }

    // publish rtrs topic
    switch (m_state)
    {
    case eState::Ready:
    case eState::Move:
    case eState::Stop:
    {
        m_rtrsTime += deltaSeconds;
        if (m_rtrsTime > 0.2f)
        {
            m_rtrsTime = 0.f;

            m_rtrs_msg.state1 = (int)m_state;
            m_rtrs_msg.state2 = 0;
            m_rtrs_msg.state3 = 0;
            m_rtrs_msg.state4 = 0;
            m_rtrs_msg.time = 0.0;
            m_rtrs_pub->publish(m_rtrs_msg);
        }
    }
    break;
    }
    //~

    return true;
}


// get task from server
cRobotClient::SharedFuture cRobotClient::ReqLogin()
{
    // RCLCPP_WARN(get_logger(), "Request Login");
    m_request->type = (int)robot::eCommandType::ReqLogin;
    m_request->name = m_name;
    return m_client->async_send_request(m_request
        , std::bind(&cRobotClient::client_response_callback, this
             , std::placeholders::_1));
}


// get task from server
cRobotClient::SharedFuture cRobotClient::EmptyWork()
{
    // RCLCPP_WARN(get_logger(), "EmptyWork");
    m_request->type = (int)robot::eCommandType::EmptyWork;
    m_request->name = m_name;
    return m_client->async_send_request(m_request
        , std::bind(&cRobotClient::client_response_callback, this
             , std::placeholders::_1));
}


// robot move
void cRobotClient::MoveRobot(const float &linear_x, const float &angular_z) 
{
    m_twist_msg.linear.x = linear_x;
    m_twist_msg.angular.z = angular_z;
    m_twist_pub->publish(m_twist_msg);
}


// robot stop
void cRobotClient::StopRobot() 
{
    m_twist_msg.linear.x = 0.0;
    m_twist_msg.angular.z = 0.0;
    m_twist_pub->publish(m_twist_msg);

    RCLCPP_INFO(get_logger(), "Stop Robot and make Node FREE!");
}


// robot service RobotCmd packet response
void cRobotClient::server_response_callback(std::shared_ptr<RobotCmd::Request> request,
                        std::shared_ptr<RobotCmd::Response> response) 
{
 //   RCLCPP_INFO(get_logger(), "Receive RobotCmd %d", request->type);
    const robot::eCommandType type = (robot::eCommandType)request.get()->type;
    switch (type)
    {
    case robot::eCommandType::AckLogin:
    {
        if (request.get()->ui0.size() < 1)
        {
            response.get()->result = 0;
            break; // error
        }

        // error occurred, clear and reconnect
        if (1 != request.get()->ui0[0])
        {
            m_state = eState::ConnectService;
            response.get()->result = 1;
        }
        else
        {
            response.get()->result = 0;
        }
    }
    break;

    case robot::eCommandType::ReqRemoteMove:
    {
        response.get()->type = (int)robot::eCommandType::AckRemoteMove;
        if (eState::Ready != m_state)
        {
            response.get()->result = 0;
            break; // error
        }

        if (request.get()->f0.size() < 3)
        {
            response.get()->result = 0;
            break; // error
        }

        m_timeDuration = request.get()->f0[0];
        m_linearVelX =  request.get()->f0[1];
        m_angularVelZ =  request.get()->f0[2];
        m_state = eState::Move;

        RCLCPP_INFO(get_logger()
            , "\nTime Duration : %f\nLinear X Cmd : %f\nAngular Z Cmd : %f",
            m_timeDuration, m_linearVelX, m_angularVelZ);

        response.get()->result = 1;
    }
    break;
    }
}


// robotserver -> client response callback
void cRobotClient::client_response_callback(SharedFuture result)
{
    RCLCPP_INFO(get_logger(), "client response type = %d", result.get()->type);

    const robot::eCommandType type = (robot::eCommandType)result.get()->type;
    switch (type)
    {
    case robot::eCommandType::AckLogin:
    {
        if (1 == result.get()->result)
        {
            RCLCPP_INFO(get_logger(), "Ack Login result = %d", result.get()->result);
            m_state = eState::Ready;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service login");
            m_state = eState::Error;
            return;
        }
    }
    break;

    case robot::eCommandType::AckEmptyWork:
    {
        RCLCPP_INFO(get_logger(), "Ack Empty Work result = %d", result.get()->result);
    }
    break;

    default:
        RCLCPP_INFO(get_logger(), "unknown response result = %d", result.get()->result);
        break;
    }
}


// robot odometry subscriber
void cRobotClient::odom_callback(const Odom::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "x=%f, y=%f, z=%f"
    //     , msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    m_rtrs_msg.posx = msg->pose.pose.position.x;
    m_rtrs_msg.posy = msg->pose.pose.position.y;
    m_rtrs_msg.posz = msg->pose.pose.position.z;
    m_rtrs_msg.orientx = msg->pose.pose.orientation.x;
    m_rtrs_msg.orienty = msg->pose.pose.orientation.y;
    m_rtrs_msg.orientz = msg->pose.pose.orientation.z;
    m_rtrs_msg.orientw = msg->pose.pose.orientation.w;

}
