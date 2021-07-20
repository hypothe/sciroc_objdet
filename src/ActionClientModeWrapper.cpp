#include "sciroc_objdet/ActionClientModeWrapper.h"

ActionClientModeWrapper::ActionClientModeWrapper()
{
  bool servers_up = false;

  

  while(!servers_up)
  {
    servers_up = true;
    servers_up = servers_up && enum_ac_.waitForServer(ros::Duration(1.0));
    servers_up = servers_up && clas_ac_.waitForServer(ros::Duration(1.0));
    servers_up = servers_up && comp_ac_.waitForServer(ros::Duration(1.0));
  }
}