//
// 2022-06-10, jjuiddong
// robot client class
//
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/robot_cmd.hpp"
#include "robot_interfaces/msg/rtrs.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using Twist = geometry_msgs::msg::Twist;
using RobotCmd = robot_interfaces::srv::RobotCmd;
using Rtrs = robot_interfaces::msg::Rtrs;
//using AmclPose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Odom = nav_msgs::msg::Odometry;


class cRobotClient : public rclcpp::Node
{
public:
    typedef rclcpp::Client<RobotCmd>::SharedFuture SharedFuture;

    cRobotClient(const string &name);
    virtual ~cRobotClient();

    bool Init();
    bool Update(const float deltaSeconds);
    SharedFuture ReqLogin();
    SharedFuture EmptyWork();


protected:
    void MoveRobot(const float &linear_x, const float &angular_z);
    void StopRobot();
    void client_response_callback(SharedFuture future);
    void server_response_callback(std::shared_ptr<RobotCmd::Request> request
        , std::shared_ptr<RobotCmd::Response> response);
    void odom_callback(const Odom::SharedPtr msg);


public:
    enum class eState {
        None,
        Initialize,
        ConnectService,
        Login, // try login
        Ready, // ready to work
        Move,
        Stop,
        Error,
    };

    eState m_state;
    string m_name;
    rclcpp::Service<RobotCmd>::SharedPtr m_service; // receive robot command
    rclcpp::Client<RobotCmd>::SharedPtr m_client; // request robot command
    std::shared_ptr<RobotCmd::Request> m_request;
    rclcpp::Publisher<Twist>::SharedPtr m_twist_pub;
    rclcpp::Publisher<Rtrs>::SharedPtr m_rtrs_pub;
    rclcpp::Subscription<Odom>::SharedPtr m_pose_sub; // Odom subscriber
    Twist m_twist_msg;
    Rtrs m_rtrs_msg;
    float m_emptyWorkTime;
    float m_timeDuration;
    float m_linearVelX;
    float m_angularVelZ;
    float m_rtrsTime;
};
