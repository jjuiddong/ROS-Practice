//
// 2022-06-04, jjuiddong
// ros robot agent class
//  - connect cluster server, send/recv cloud packet
//
#include "robot_interfaces/srv/robot_cmd.hpp"
#include "robot_interfaces/msg/rtrs.hpp"
#include "rclcpp/rclcpp.hpp"
using RobotCmd = robot_interfaces::srv::RobotCmd;
using Rtrs = robot_interfaces::msg::Rtrs;


class cControlServer;

class cRobot : public rclcpp::Node 
             , public robot::s2c_ProtocolHandler
{
public:
    cRobot(const string &name);
    virtual ~cRobot();

    bool Init(cControlServer *ctrlSvr);
    bool Update(const float deltaSeconds);
    bool ConnectCluster();
    bool SendRequest();
    void Clear();


protected:
    void sub_callback(const Rtrs::SharedPtr msg);

    // robot protocol handler
	virtual bool Welcome(robot::Welcome_Packet &packet) override;
	virtual bool AckLogin(robot::AckLogin_Packet &packet) override;
	virtual bool AckLogout(robot::AckLogout_Packet &packet) override;
	virtual bool AckPathData(robot::AckPathData_Packet &packet) override;
	virtual bool AckInitializeEnd(robot::AckInitializeEnd_Packet &packet) override;
	virtual bool ReqMovable(robot::ReqMovable_Packet &packet) override;
	virtual bool ReqMove(robot::ReqMove_Packet &packet) override;
	virtual bool ReqMoveCancel(robot::ReqMoveCancel_Packet &packet) override;
	virtual bool ReqWait(robot::ReqWait_Packet &packet) override;
	virtual bool ReqEmergencyStop(robot::ReqEmergencyStop_Packet &packet) override;
	virtual bool AckWork(robot::AckWork_Packet &packet) override;
	virtual bool ReqUntil(robot::ReqUntil_Packet &packet) override;
	virtual bool ReqMessage(robot::ReqMessage_Packet &packet) override;


public:
    enum class eState {
        None, 
        Ready,  // ready to connect
        TryConnect,  // connecting
        Connect,  // success connect
        Stop, // disconnect
    };

    eState m_state;
    string m_name; // robot name
    cControlServer *m_ctrlSvr; // reference
    int m_rtrsPort;
    network2::cTcpClient m_tcp;
    network2::cUdpClient m_udp;
    robot::c2s_Protocol m_protocol;
    rtrs::c2s_Protocol m_rtrsProtocol;
    rclcpp::Client<RobotCmd>::SharedPtr m_client; // request robot command
    std::shared_ptr<RobotCmd::Request> m_request;
    rclcpp::Subscription<Rtrs>::SharedPtr m_rtrs_sub; // rtrs subscriber    
};
