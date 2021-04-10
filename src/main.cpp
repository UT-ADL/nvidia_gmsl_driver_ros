#include <iostream>
#include <ros/ros.h>
#include "SekonixCamera.h"
#include "PrintEventHandler.h"

int main(int argc, char **argv) {
  std::cout << "------------------ main --------------------" << std::endl;
  ros::init(argc, argv, "sekonix_camera_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  auto signal_handler = [](int sig) {
    (void) sig;
    std::cout << "Signal handler calling...." << std::endl;
    ros::shutdown();
  };

  // Detect exit signals
  signal(SIGHUP, signal_handler);  // controlling terminal closed, Ctrl-D
  signal(SIGINT, signal_handler);  // Ctrl-C
  signal(SIGQUIT, signal_handler); // Ctrl-\, clean quit with core dump
  signal(SIGABRT, signal_handler); // abort() called.
  signal(SIGTERM, signal_handler); // kill command
  signal(SIGSTOP, signal_handler); // kill command

  std::string name_pretty{"Main"};
  PrintEventHandler::Ptr print_event_handler = std::make_shared<PrintEventHandler>();
  print_event_handler->Print(name_pretty, "sekonix_camera_node started!");

  SekonixCamera sekonix_camera(nh_p, print_event_handler);

  ros::AsyncSpinner spinner(1);
  print_event_handler->Print(name_pretty, "Spinning has started!");
  spinner.start();

  ros::waitForShutdown();
  print_event_handler->Print(name_pretty, "Spinning has ended!");

  sekonix_camera.Shutdown();

  print_event_handler->Print(name_pretty, "Just before return 0;");
  std::cout << "------------------ end main --------------------" << std::endl;
  return 0;
}