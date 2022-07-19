//
// 2022-06-04, jjuiddong
// ros robot server
//  - connect to webserver
//  - task management with webserver, robot
//
#include "stdafx.h"
#include "ctrlsvr/controlserver.h"

bool g_loop = true;

void RosServerThread() 
{
  cControlServer::sInitParam param;
  param.id = "guest3";
  param.passwd = "1111";
  //param.url = "techsocietyrelay.cafe24app.com:80";
  param.url = "192.168.0.19:8002";

  auto svr = std::make_shared<cControlServer>();
  if (!svr->Init(param))
    return;

  cTimer m_timer;
  while (g_loop)
  {
    const double dt = m_timer.GetDeltaSeconds();

    if (!svr->Update(dt))
      break; // finish
    rclcpp::spin_some(svr);
    usleep(1000); // 1 milliseconds sleep
  }
}


void my_handler(int s)
{
    printf("Caught signal %d\n",s);
    //exit(1); 
    g_loop = false; // finish program
}


int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);

  // ctrl+c key handling
   struct sigaction sigIntHandler;
   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;
   sigaction(SIGINT, &sigIntHandler, NULL);
  //~

  std::thread th1 = std::thread(RosServerThread);
  th1.join();
  rclcpp::shutdown();
  std::cout << "finish ros agent server\n";
}
