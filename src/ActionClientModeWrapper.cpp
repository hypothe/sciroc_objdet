#include "sciroc_objdet/ActionClientModeWrapper.h"

class ActionClientModeWrapper::wait_visitor : public boost::static_visitor<bool>
{
  public:
    bool operator()(NNNPtr c) const {  return true;  }
    bool operator()(OEAPtr c) const {  return c->waitForServer(wait_time); }
    bool operator()(OKAPtr c) const {  return c->waitForServer(wait_time); }
    bool operator()(OCAPtr c) const {  return c->waitForServer(wait_time); }

  private:
    ros::Duration wait_time = ros::Duration(1.0);
};

class ActionClientModeWrapper::send_goal_visitor : public boost::static_visitor<>
{
	public:
		send_goal_visitor(ActionClientModeWrapper &p) : parent(p){}
		void operator()(NNNPtr c) const 
		{
			boost::unique_lock<boost::shared_mutex> lockInitResult(parent.mutexResult_);
			parent.state_ = actionlib::SimpleClientGoalState::SUCCEEDED;
			parent.n_found_tags_ = 0;
			parent.found_tags_.clear();
			parent.expected_tags_.clear();
		}

		void operator()(OEAPtr c) const
		{
			{
				boost::unique_lock<boost::shared_mutex> lockGoalResult(parent.mutexResult_);
				parent.state_ = actionlib::SimpleClientGoalState::ACTIVE;
			}
			sciroc_objdet::ObjectEnumerationGoal goal;
			goal.table_pos = parent.table_pos_;
			c->sendGoal(goal, boost::bind(static_cast<void (ActionClientModeWrapper::*)(const actionlib::SimpleClientGoalState &, const sciroc_objdet::ObjectEnumerationResultConstPtr &)>(&ActionClientModeWrapper::doneCB), &parent, _1, _2),
									OEA::SimpleActiveCallback(), OEA::SimpleFeedbackCallback());
		}

		void operator()(OKAPtr c) const
		{
			{
				boost::unique_lock<boost::shared_mutex> lockGoalResult(parent.mutexResult_);
				parent.state_ = actionlib::SimpleClientGoalState::ACTIVE;
			}
			sciroc_objdet::ObjectClassificationGoal goal;
			goal.table_pos = parent.table_pos_;
			
			c->sendGoal(goal, boost::bind(static_cast<void (ActionClientModeWrapper::*)
														(const actionlib::SimpleClientGoalState& ,
														const sciroc_objdet::ObjectClassificationResultConstPtr& )>(&ActionClientModeWrapper::doneCB),
													&parent, _1, _2),
												OKA::SimpleActiveCallback(), OKA::SimpleFeedbackCallback());
		}

		void operator()(OCAPtr c) const
		{
			{
				boost::unique_lock<boost::shared_mutex> lockGoalResult(parent.mutexResult_);
				parent.state_ = actionlib::SimpleClientGoalState::ACTIVE;
			}
			sciroc_objdet::ObjectComparisonGoal goal;
			{
				boost::shared_lock<boost::shared_mutex> lockGoalResult(parent.mutexResult_);
				goal.expected_tags = parent.expected_tags_;
			}
			goal.table_pos = parent.table_pos_;
			c->sendGoal(goal, boost::bind(static_cast<void (ActionClientModeWrapper::*)
														(const actionlib::SimpleClientGoalState& ,
														const sciroc_objdet::ObjectComparisonResultConstPtr& )>(&ActionClientModeWrapper::doneCB),
													&parent, _1, _2),
												OCA::SimpleActiveCallback(), OCA::SimpleFeedbackCallback());
		}
	private:
		ActionClientModeWrapper& parent;
};

void ActionClientModeWrapper::doneCB(const actionlib::SimpleClientGoalState &state,
								const sciroc_objdet::ObjectEnumerationResultConstPtr &result)
{
	boost::unique_lock<boost::shared_mutex> lockGoalResult(mutexResult_);
	state_ = state;
	n_found_tags_ = result->n_found_tags;
	ROS_DEBUG("[FFTT]: FOUND TAGS %d", n_found_tags_);
}

void ActionClientModeWrapper::doneCB(const actionlib::SimpleClientGoalState &state,
								const sciroc_objdet::ObjectClassificationResultConstPtr &result)
{
	boost::unique_lock<boost::shared_mutex> lockGoalResult(mutexResult_);
	state_ = state;
	found_tags_ = result->found_tags;
	n_found_tags_ = result->found_tags.size();
}

void ActionClientModeWrapper::doneCB(const actionlib::SimpleClientGoalState &state,
								const sciroc_objdet::ObjectComparisonResultConstPtr &result)
{
	boost::unique_lock<boost::shared_mutex> lockGoalResult(mutexResult_);
	state_ = state;
	found_tags_ = result->found_tags;
	n_found_tags_ = result->found_tags.size();

	std::sort(found_tags_.begin(), found_tags_.end());
	std::sort(expected_tags_.begin(), expected_tags_.end());
	match_ = std::includes(found_tags_.begin(), found_tags_.end(),
																expected_tags_.begin(), expected_tags_.end());
	expected_tags_.clear();
}

class ActionClientModeWrapper::cancel_goal_visitor : public boost::static_visitor<void>
{
  public:
		cancel_goal_visitor(ActionClientModeWrapper &p) : parent(p){}
    void operator()(NNNPtr c) const
		{
			boost::unique_lock<boost::shared_mutex> lockGoalResult(parent.mutexResult_);
			parent.state_ = actionlib::SimpleClientGoalState::PREEMPTED;  
		}
    void operator()(OEAPtr c) const {	c->cancelGoal();	}
    void operator()(OKAPtr c) const {	c->cancelGoal();	}
    void operator()(OCAPtr c) const {	c->cancelGoal();	}
		
	private:
		ActionClientModeWrapper& parent;
};

class ActionClientModeWrapper::get_state_visitor : public boost::static_visitor<actionlib::SimpleClientGoalState>
{
  public:
  	actionlib::SimpleClientGoalState operator()(NNNPtr c) const {  return actionlib::SimpleClientGoalState::SUCCEEDED;  }
  	actionlib::SimpleClientGoalState operator()(OEAPtr c) const {  return c->getState(); }
  	actionlib::SimpleClientGoalState operator()(OKAPtr c) const {  return c->getState(); }
  	actionlib::SimpleClientGoalState operator()(OCAPtr c) const {  return c->getState(); }
};
/* TEST
*/

ActionClientModeWrapper::ActionClientModeWrapper(int mode)
: ObjDetMode(mode),
// TODO: get action names from rosparam
	state_(actionlib::SimpleClientGoalState::StateEnum::LOST),
	match_(false), n_found_tags_(0)
{
	std::string enum_name, clas_name, comp_name;
	ros::param::param("/sciroc_darknet_bridge/objdet/actions/enumeration/topic", enum_name, std::string("/enum_bridge_as"));
	ros::param::param("/sciroc_darknet_bridge/objdet/actions/classification/topic", clas_name, std::string("/clas_bridge_as"));
	ros::param::param("/sciroc_darknet_bridge/objdet/actions/comparison/topic", comp_name, std::string("/comp_bridge_as"));

	ROS_INFO("%s\n%s\n%s", enum_name.c_str(), clas_name.c_str(), comp_name.c_str());

	enum_ac_ = std::make_shared<OEA>(enum_name);
	clas_ac_ = std::make_shared<OKA>(clas_name);
	comp_ac_ = std::make_shared<OCA>(comp_name);

	ac_.insert(std::pair<Mode, OXAPtr>(Mode::NONE, nullptr));
	ac_.insert(std::pair<Mode, OXAPtr>(Mode::ENUMERATE, enum_ac_));
  ac_.insert(std::pair<Mode, OXAPtr>(Mode::CLASSIFY, clas_ac_));
  ac_.insert(std::pair<Mode, OXAPtr>(Mode::COMPARE, comp_ac_));
  /*
	*/
}

// If no mode is expressed insert -1, which will be converted to a Mode::NONE
// by the ObjDetModed constructor
ActionClientModeWrapper::ActionClientModeWrapper()
: ActionClientModeWrapper::ActionClientModeWrapper(-1){}

/*	Wait only for the currently tracked action server	*/
void ActionClientModeWrapper::waitForServer()
{
	bool server_up = false;
	std::stringstream ss;
	ss << static_cast<ObjDetMode>(*this);
	while (!server_up)
	{
		server_up = boost::apply_visitor(ActionClientModeWrapper::wait_visitor(), ac_[mode_]);
		ROS_WARN("Waiting for objdet %s action server.", ss.str().c_str());
	}

}
/*	Wait for all the action servers tracked to be up	*/
void ActionClientModeWrapper::waitForAllServers()
{
  bool servers_up = false;

	while(!servers_up)
  {
    servers_up = true;
    for (auto mode_ac : ac_)
    {
			servers_up = servers_up && boost::apply_visitor(ActionClientModeWrapper::wait_visitor(), mode_ac.second);
		}
		ROS_WARN("Waiting for all objdet action servers.");
	}
}

void ActionClientModeWrapper::sendGoal()
{
	{
		boost::unique_lock<boost::shared_mutex> lockGoalResult(mutexResult_);
		found_tags_.clear(); //cleanup
		match_ = false;
		n_found_tags_ = 0;
	}

	ActionClientModeWrapper::send_goal_visitor sgv = ActionClientModeWrapper::send_goal_visitor(*this);

	boost::apply_visitor(sgv, ac_[mode_]);
}
void ActionClientModeWrapper::cancelGoal()
{
	{
		boost::unique_lock<boost::shared_mutex> lockGoalResult(mutexResult_);
		found_tags_.clear(); //cleanup
		match_ = false;
		n_found_tags_ = 0;
	}
	/*	TODO: should we also clear the expected tags?*/

	ActionClientModeWrapper::cancel_goal_visitor cgv = ActionClientModeWrapper::cancel_goal_visitor(*this);

	boost::apply_visitor(cgv, ac_[mode_]);

}

actionlib::SimpleClientGoalState ActionClientModeWrapper::getState()
{
	auto tmp_state = boost::apply_visitor(ActionClientModeWrapper::get_state_visitor(), ac_[mode_]);
	{
		boost::unique_lock<boost::shared_mutex> lockGoalResult(mutexResult_);
		state_ = tmp_state;
	}
	return state_;
}
bool ActionClientModeWrapper::getMatch() 	{ return match_; }
int ActionClientModeWrapper::getNumTags() { return n_found_tags_; }
std::vector<std::string> ActionClientModeWrapper::getExpectedTags() { return expected_tags_; }
std::vector<std::string> ActionClientModeWrapper::getFoundTags() 		{ return found_tags_; }

void ActionClientModeWrapper::setExpectedTags(std::vector<std::string> expected_tags)
{
	expected_tags_ = expected_tags;
}

void ActionClientModeWrapper::setTablePos(geometry_msgs::Point table_pos)
{
	table_pos_ = table_pos;
}

std::string ActionClientModeWrapper::getText()
//std::ostream& operator<<(std::ostream& os, ActionClientModeWrapper o)
{
	std::stringstream ss;
	ss << static_cast<ObjDetMode>(*this);
	
	ss << "Mode: " << ss.str() << std::endl;

	ss << "State: " << getState().toString() << std::endl;

	ss << "Expected tags: ";
	for (auto tag : getExpectedTags())
		ss << tag << ", ";
	ss << std::endl;

	ss << "Found tags: ";
	for (auto tag : getFoundTags())
		ss << tag << ", ";
	ss << std::endl;
	
	ss << "Match: " << std::boolalpha << getMatch() << std::endl;

	return ss.str();
}