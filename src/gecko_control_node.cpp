/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * gecko_control_node.cpp
 *
 *  This node subscribes reference velocity data from higher
 * level controller, such as wall_follower_node, and
 * generates reference velocity data for micro-controllers
 * with some modifications.
 *
 * Author: Masahiro Katsumoto
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <ros/ros.h>
#include "gecko_control/GeckoControl.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gecko_control_node");

  GeckoControl gecko_control;

  ros::Rate loop_rate_Hz(10);
  while(ros::ok())
  {
   ros::spinOnce();
   loop_rate_Hz.sleep();
  }
  return 0;
}
