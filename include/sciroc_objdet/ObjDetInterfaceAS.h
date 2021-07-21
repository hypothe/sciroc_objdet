#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "sciroc_objdet/ObjDetInterfaceAction.h"
#include "sciroc_objdet/ActionClientModeWrapper.h"

#include <sstream>
#include <memory>

/*
  This class will have a timer, started whan a goal is received, and which will
  periodically check for preemption requests or success of the called actions.

  Once a goal is received, it is "forwarded" to the "inner" action servers.
  If a preempt request is received for this goal (checked in the timer callback) it
  is as well forwarded to the active "inner" AS.
*/

/*
	TODO 22-07-21

	- generate the three action servers nodes
*/

class ObjDetInterfaceAS
{
public:
	ObjDetInterfaceAS(std::string name);
/*
  ~ObjDetInterfaceAS(void)
*/
	void goalCB();
	void preemptCB();
	void clock_Cllbck(const ros::TimerEvent &);

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sciroc_objdet::ObjDetInterfaceAction> as_;
  std::string action_name_;
  sciroc_objdet::ObjDetInterfaceFeedback feedback_;
  sciroc_objdet::ObjDetInterfaceResult result_;

  ros::Timer ODI_clock;
	const double ODI_clock_cycle = 0.02;
	std::shared_ptr<ActionClientModeWrapper> action_client_;
};
