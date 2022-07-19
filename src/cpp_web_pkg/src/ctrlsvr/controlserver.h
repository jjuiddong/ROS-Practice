//
// 2022-06-03, jjuiddong
// ros robot control server
//  - communicate with cluster server
//  - receive robot data
//
#pragma once

#include "robot_interfaces/srv/robot_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
using RobotCmd = robot_interfaces::srv::RobotCmd;


class cRobot;
class cControlServer : public rclcpp::Node 
                    , public webrelay::s2c_ProtocolHandler
{
public:
    typedef rclcpp::Client<RobotCmd>::SharedFuture SharedFuture;

    struct sInitParam 
    {
        string id; // user id
        string passwd; // user password
        string url; // web relay server url
    };

    struct sTask
    {
        string robotName;
        robot::eCommandType type;
    };

    cControlServer();
    virtual ~cControlServer();

    bool Init(const sInitParam &param);
    bool Update(const float deltaSeconds);
    void Clear();


protected:
    cRobot* FindSession(const string &robotName);
    bool ReConnect();

    // webrelay protocol handler
	virtual bool Welcome(webrelay::Welcome_Packet &packet) override;
	virtual bool AckLogin(webrelay::AckLogin_Packet &packet) override;
	virtual bool AckLogout(webrelay::AckLogout_Packet &packet) override;
	virtual bool ReqConnectProxyServer(webrelay::ReqConnectProxyServer_Packet &packet) override;

    // RobotCmd Service
    void response_callback(std::shared_ptr<RobotCmd::Request> request,
                            std::shared_ptr<RobotCmd::Response> response);

public:
    void client_response_callback(SharedFuture result);


public:
    enum class eState {
        None, //0
        TryConnect, //1 try connect relay server
        SyncCluster, //2 communication with cluster server
        WaitCluster, //3 wait launch cluster server
        Stop, //4
    };

    eState m_state;
    sInitParam m_param;
    string m_clusterIp; // cluster server ip
    int m_clusterPort; // cluster server port

    network2::cNetController m_netController;
    network2::cWebClient m_client;
    webrelay::c2s_Protocol m_protocol;
    string m_relaySvrUrl; // relay server url
    float m_reconTime; // reconnect time
    float m_connTime; // connect waitting time (to relay server)

    vector<std::shared_ptr<cRobot>> m_robots;
    rclcpp::Service<RobotCmd>::SharedPtr m_service; // ros service

    queue<sTask> m_tasks;
};
