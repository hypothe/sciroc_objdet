#include "ros/ros.h"
#include "sciroc_objdet/ObjDetInterfaceAS.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "objdet_interface");
  ObjDetInterfaceAS objdet_interface(ros::this_node::getName());
  ros::spin();

  return 0;
}