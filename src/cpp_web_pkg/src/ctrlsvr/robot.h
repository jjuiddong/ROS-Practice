//
// 2022-06-04, jjuiddong
// ros robot agent class
//  - connect cluster server, send/recv cloud packet
//
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/robot_cmd.hpp"
#include "robot_interfaces/msg/rtrs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"

using RobotCmd = robot_interfaces::srv::RobotCmd;
using Rtrs = robot_interfaces::msg::Rtrs;
using LaserScan = sensor_msgs::msg::LaserScan;
using Image = sensor_msgs::msg::Image;


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
    void rtrs_callback(const Rtrs::SharedPtr msg);
    void scan_callback(const LaserScan::SharedPtr msg);
    void image_callback(const Image::SharedPtr msg);

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
    rclcpp::Subscription<LaserScan>::SharedPtr m_scan_sub; // Scan subscriber
    rclcpp::Subscription<Image>::SharedPtr m_image_sub; // camera image subscriber

    rclcpp::Time m_sendImageTime; // seconds
    unsigned char *m_imgBuffer;
};
