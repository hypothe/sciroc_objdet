#include "sciroc_objdet/ObjDetInterfaceAS.h"

ObjDetInterfaceAS::ObjDetInterfaceAS(ros::NodeHandle nodeHandle, std::string name) :
  nh_(nodeHandle),
  as_(nh_, name, false),
  action_name_(name),
  action_client_(std::make_shared<ActionClientModeWrapper>())
{
  ODI_clock = nh_.createTimer(ros::Duration(ODI_clock_cycle), &ObjDetInterfaceAS::clock_Cllbck, this, false, false);
  action_client_->waitForAllServers();
  as_.registerGoalCallback(boost::bind(&ObjDetInterfaceAS::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ObjDetInterfaceAS::preemptCB, this));
  as_.start();
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

  std::string table_id = goal->table_id;
  action_client_->setTablePos(getTablePos(table_id));

  /*  Here select which of the "inner" action server to call and prepare the message  */
  action_client_->setMode(mode);

  feedback_.step = "RECEIVED";
  as_.publishFeedback(feedback_);


 // Generic Action Client handling the three possible modes autonomously

  // action_client_->waitForServer();

  ROS_INFO("%s: %s", action_name_.c_str(), action_client_->getText().c_str());
  /*  Prepare the client and send the goal  */
  action_client_->setExpectedTags(goal->expected_tags);
  action_client_->sendGoal();

  feedback_.step = "ACCEPTED";
  as_.publishFeedback(feedback_);

  ODI_clock.start();
}

void ObjDetInterfaceAS::preemptCB()
{
  ODI_clock.stop();
  /*  WARN: The clock is stopped right now, the callback is
      stopped being served, hence the status is not tracked anymore
      as soon as a preempt request is received (not a problem when we
      have only a goal served at any given time)
   */
  ROS_DEBUG("%s: Preemption request received", action_name_.c_str());
  
  /*  Wait until the goal of the chosen inner server results canceled,
      then preempt this one too
  */
  action_client_->cancelGoal();
  while (!(action_client_->getState().isDone()))
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
  if (!as_.isActive())
  {
    return;
  }
  /*  TODO: possibly retrieve feedback from the
      inner action servers and publish it instead
      of a "meaningless" ONGOING
  */
  ROS_DEBUG("[ObjDetAS]: clock, about to get state");
  if (!(action_client_->getState().isDone()))
  {
    feedback_.step = "ONGOING";
    as_.publishFeedback(feedback_);
    // ROS_INFO("ac: %s", action_client_->getState().toString().c_str());
    return;
  }
  ROS_DEBUG("Action client is done");
  /*
  std::stringstream s_mode;
  s_mode << static_cast<ActionClientModeWrapper>(*action_client_);
  ROS_DEBUG("Action Client %s done", s_mode.str().c_str());
*/
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
  feedback_.step = action_client_->getState().toString();
  as_.publishFeedback(feedback_);
  
  ODI_clock.stop();

}

geometry_msgs::Point ObjDetInterfaceAS::getTablePos(std::string table_id)
{
  geometry_msgs::Point table_pos;

  // If the POI id is not set there could be an error, or might simply not be
  // used for debugging purposes. Throw a soft warn.
  if (table_id.empty())
  {
    ROS_WARN("[%s]: empty 'table_id' field in goal request", action_name_.c_str());
    return table_pos;
  }
  std::string param_id = "/mmap/poi/submap_0/" + table_id + "_objdet";
  if (!nh_.hasParam(param_id))
  {
    ROS_ERROR("[%s]: no parameter %s found", action_name_.c_str(), param_id.c_str());
    return table_pos;
  }
  
  XmlRpc::XmlRpcValue poi_data;
  nh_.getParam(param_id, poi_data); // can always be retrieved
  ROS_ASSERT(poi_data.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  ROS_ASSERT(poi_data[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(poi_data[3].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  table_pos.x = static_cast<double>(poi_data[2]);
  table_pos.y = static_cast<double>(poi_data[3]);

  return table_pos;
}