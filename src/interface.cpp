#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "sciroc_objdet/ObjDetInterfaceAction.h"

#include <sstream>
#include <memory>
#include <stdlib.h>
#include <time.h>

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
    ODI_clock = nh_.createTimer(ros::Duration(0.02), &ObjDetInterfaceAction::clock_Cllbck, this, false, false);

    as_.registerGoalCallback(boost::bind(&ObjDetInterfaceAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ObjDetInterfaceAction::preemptCB, this));
    as_.start();

		/*	TODO	*/
		srand(time(NULL));
	}
/*
  ~ObjDetInterfaceAction(void)
  {
  }
*/

  void goalCB()
  {
		std::stringstream info;
		auto goal = as_.acceptNewGoal();

    if (as_.isPreemptRequested()){  preemptCB();  }
    /*  Pending preemption might not trigger the preempt callback in the
        time window of acceptNewGoal() (**see that command reference**)
    */

    int mode = goal->mode;
    std::cout << mode << std::endl;
    std::vector<std::string> exp_tags = goal->expected_tags;
    //boost::array<std::string, 3> exp_tags = goal->expected_tags;

    info << "\t<" << mode << ">\n";
    for (auto tag:exp_tags){  info << "\t- " << tag << "\n";  }

    ROS_DEBUG("%s: New goal received\n%s", action_name_.c_str(), info.str().c_str());

    /*  Here select which of the "inner" action server to call and prepare the message  */
    switch (mode)
    {
      case 0:
        /* TODO */
        ROS_DEBUG("Object ENUMERATION");
        break;
      case 1:
        /* TODO */
        ROS_DEBUG("Object CLASSIFICATION");
        break;
      case 2:
        /* TODO */
        ROS_DEBUG("Object COMPARISON");
        break;

      default:
        ROS_WARN("%s: Unexpected mode [%d]", action_name_.c_str(), mode);
        as_.setPreempted();
        return;
    }

    ODI_clock.start();
    /*  Here call such action server  */

  }

  void preemptCB()
  {
    ROS_DEBUG("%s: Preemption request received", action_name_.c_str());
		/*	TODO
				Preempt the active action client, wait for it to return,
				then set this server to be preempted as well
		*/
    as_.setPreempted();
    ODI_clock.stop();
  }

  void clock_Cllbck(const ros::TimerEvent&)
  {
    /*  Check if the action server called has returned the result:
        if so, fill the result appropriately and return it
    */

		if (!as_.isActive()){	return;	}

		/*	TODO	*/
		int ii = rand() % 100;
		feedback_.step = std::to_string(ii);
		as_.publishFeedback(feedback_);

		if (ii == 0)
		{
			ROS_INFO("Inner action server succeeded");
			as_.setSucceeded(result_);
			ODI_clock.stop();
		}
		else if(ii == 1)
		{
			ROS_INFO("Inner action server failed");
			as_.setAborted(result_);
			ODI_clock.stop();
		}
	}

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sciroc_objdet::ObjDetInterfaceAction> as_;
  std::string action_name_;
  sciroc_objdet::ObjDetInterfaceFeedback feedback_;
  sciroc_objdet::ObjDetInterfaceResult result_;

  ros::Timer ODI_clock;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "objdet_interface");
  ObjDetInterfaceAction objdet_interface(ros::this_node::getName());
  ros::spin();

  return 0;
}