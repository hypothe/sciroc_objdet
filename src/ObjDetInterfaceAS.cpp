#include "sciroc_objdet/ObjDetInterfaceAS.h"

ObjDetInterfaceAS::ObjDetInterfaceAS(std::string name) :
  as_(nh_, name, false),
  action_name_(name)
{
  ODI_clock = nh_.createTimer(ros::Duration(ODI_clock_cycle), &ObjDetInterfaceAS::clock_Cllbck, this, false, false);

  as_.registerGoalCallback(boost::bind(&ObjDetInterfaceAS::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ObjDetInterfaceAS::preemptCB, this));
  as_.start();

  /*	TODO	*/
  srand(time(NULL));
}
/*
  ~ObjDetInterfaceAS(void)
  {
  }
*/

void ObjDetInterfaceAS::goalCB()
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

  info << "\t<" << mode << ">\n";
  for (auto tag:exp_tags){  info << "\t- " << tag << "\n";  }

  ROS_DEBUG("%s: New goal received\n%s", action_name_.c_str(), info.str().c_str());
  
  feedback_.step = "RECEIVED";
  as_.publishFeedback(feedback_);

  /*  Here select which of the "inner" action server to call and prepare the message  */

  /*  TODO
    Change the switch case to an if cascade, where the mode is checked with ActionClientModeWrapper::Mode::ENUMERATE etc.
    and then the ACMW mode is set accordingly (with the foos inherited by ObjDetMode)

    After that call the waitForServer on that ACMW.
  */
 // Generic Action Client handling the three possible modes autonomously
  action_client_ = std::make_shared<ActionClientModeWrapper>(mode);
  std::stringstream s_mode;

  action_client_->waitForServer();

  s_mode << action_client_.get();
  ROS_INFO("%s: %s", action_name_.c_str(), s_mode.str().c_str());

  feedback_.step = "ACCEPTED";
  as_.publishFeedback(feedback_);

  ODI_clock.start();
  /*  Here call such action server  */

}

void ObjDetInterfaceAS::preemptCB()
{
  ODI_clock.stop();
  /*  WARN: The clock is stopped right now, the callback is
      stopped being served, hence the status is no more tracked
      as soon as a preempt request is received.
   */
  ROS_DEBUG("%s: Preemption request received", action_name_.c_str());
  
  /*  Wait until the goal of the chosen inner server results canceled,
      then preempt this one too
  */
  action_client_->cancelGoal();
  while (action_client_->getState().isDone())
  {
    ros::Duration(0.1).sleep();
  }
  as_.setPreempted();
}

void ObjDetInterfaceAS::clock_Cllbck(const ros::TimerEvent&)
{
  /*  Check if the action server called has returned the result:
      if so, fill the result appropriately and return it
  */

  if (!as_.isActive()){	return;	}

  if (!action_client_->getState().isDone())
  {
    feedback_.step = "ONGOING";
    as_.publishFeedback(feedback_);
    return;
  }

  switch(action_client_->getState().state_)
  {
    case actionlib::SimpleClientGoalState::PREEMPTED :
    {
      result_.found_tags.clear();
      result_.match = false;
      as_.setPreempted(result_);
      break;
    }
    case actionlib::SimpleClientGoalState::ABORTED :
    {
      result_.found_tags.clear();
      result_.match = false;
      as_.setAborted(result_);
      break;
    }
    case actionlib::SimpleClientGoalState::SUCCEEDED :
    {
      result_.n_found_tags = action_client_->getNumTags();
      result_.found_tags = action_client_->getFoundTags();
      result_.match = action_client_->getMatch();
      as_.setSucceeded(result_);
      break;
    }
  }
}

