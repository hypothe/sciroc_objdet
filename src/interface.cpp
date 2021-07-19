#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "sciroc_objdet/ObjDetInterfaceAction.h"

#include <sstream>
#include <memory>

/*
  This class will have a timer, started whan a goal is received, and which will
  periodically check for preemption requests or success of the called actions.

  Once a goal is received, it is "forwarded" to the "inner" action servers.
  If a preempt request is received for this goal (checked in the timer callback) it
  is as well forwarded to the active "inner" AS.
*/

class ObjDetInterfaceAction
{
public:

  ObjDetInterfaceAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    as_.registerGoalCallback(boost::bind(&ObjDetInterfaceAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ObjDetInterfaceAction::preemptCB, this));
    as_.start();

    ROS_INFO("AQ");
  }
/*
  ~ObjDetInterfaceAction(void)
  {
  }
*/

  void goalCB()
  {
    ROS_INFO("request received");
    std::stringstream info;
    auto goal = as_.acceptNewGoal();

    uint8_t mode = goal->mode;
    std::vector<std::string> exp_tags = goal->expected_tags;

    info << "\t" << mode << "\n";
    for (std::string tag:exp_tags){  info << "\t- " << tag << "\n";  }

    ROS_DEBUG("%s: New goal received\n%s", action_name_.c_str(), info.str().c_str());
  }

  void preemptCB()
  {
    ROS_DEBUG("%s: Preemption request received", action_name_.c_str());
  }

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sciroc_objdet::ObjDetInterfaceAction> as_;
  std::string action_name_;
  sciroc_objdet::ObjDetInterfaceFeedback feedback_;
  sciroc_objdet::ObjDetInterfaceResult result_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "objdet_interface");

  ObjDetInterfaceAction objdet_interface(ros::this_node::getName());
  ros::spin();

  return 0;
}