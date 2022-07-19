//
// 2022-06-10, jjuiddong
//  - robot client main
//    - connect RobotCmd Service
//    - 1. Login
//    - 2. Request Task
//
#include <chrono>
#include <cstdlib>
#include <memory>
#include <unistd.h>
#include "stdafx.h"
#include "robotcli/robot_client.h"

bool g_loop = true;

void BreakHandler(int s)
{
    printf("Caught signal %d\n",s);
    //exit(1); 
    g_loop = false;
}

int ThreadFunc() 
{
    auto robot = std::make_shared<cRobotClient>("robot1");
    if (!robot->Init())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize robot client");
        return 1;
    }

    common::cTimer timer;
    timer.Create();
    while (g_loop)
    {
        const float dt = (float)timer.GetDeltaSeconds();
        if (!robot->Update(dt))
            break;
        rclcpp::spin_some(robot);
        usleep(1000); // 1 milliseconds sleep
    }

    rclcpp::shutdown();
    return 1;
}

// main entry point
int main(int argc, char **argv)
{
    std::cout << "start robot client" << std::endl;

    // ctrl+c key handling
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = BreakHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    //~

    rclcpp::init(argc, argv);
    std::thread th = std::thread(ThreadFunc);
    th.join();

    // // Request Login to RosControl Server
    // auto result = robot->ReqLogin();
    // if (rclcpp::spin_until_future_complete(robot, result) ==
    //     rclcpp::executor::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result : %s",
    //                 (result.get()->result == 1) ? "True" : "False");

    //     if ((int)robot::eCommandType::AckLogin == result.get()->type)
    //     {
    //         RCLCPP_INFO(robot->get_logger(), "Ack Login result = %d", result.get()->result);
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service login");
    //         rclcpp::shutdown();
    //         return 0;
    //     }
    // }
    // else
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service login");
    //     rclcpp::shutdown();
    //     return 0;
    // }
    // //~Login

    // // Request Task
    // while (1)
    // {
    //     result = robot->GetTask();
    //     if (rclcpp::spin_until_future_complete(robot, result) ==
    //         rclcpp::executor::FutureReturnCode::SUCCESS)
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result : %s", 
    //         (result.get()->result == 1) ? "True" : "False");

    //         if ((int)robot::eCommandType::AckEmptyWork == result.get()->type)
    //         {
    //                 // auto t_start = robot->now();
    //                 // auto t_now = robot->now();

    //                 // if (result.get()->i0.empty())
    //                 //   break; // error
    //                 // if (result.get()->f0.size() < 2)
    //                 //   break; // error

    //                 // const int time_duration =  result.get()->i0[0];
    //                 // const float linear_vel_x =  result.get()->f0[0];
    //                 // const float angular_vel_z =  result.get()->f0[1];

    //                 // auto t_delta = time_duration * 1e9;

    //                 // RCLCPP_INFO(robot->get_logger()
    //                 //   , "\nTime Duration : %d\nLinear X Cmd : %f\nAngular Z Cmd : %f",
    //                 //   time_duration, linear_vel_x, angular_vel_z);
    //                 // RCLCPP_INFO(robot->get_logger(), "Request Received Robot Starts to Move");

    //                 // while ((t_now - t_start).nanoseconds() < t_delta) {
    //                 //   t_now = robot->now();
    //                 //   robot->move_robot(linear_vel_x, angular_vel_z);
    //                 // }
    //                 // robot->stop_robot();
    //         }

    //             sleep(3);
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    //         break;
    //     }
    // }//~


    std::cout << "finish robot client\n";
    return 0;
}
