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
			parent.state_ = actionlib::SimpleClientGoalState::SUCCEEDED;
			parent.n_found_tags_ = 0;
			parent.found_tags_.clear();
			parent.expected_tags_.clear();
		}

		void operator()(OEAPtr c) const
		{
			sciroc_objdet::ObjectEnumerationGoal goal;
			auto doneCB =
			[&](const actionlib::SimpleClientGoalState &state, const sciroc_objdet::ObjectEnumerationResultConstPtr &result)
			{
				parent.state_ = state;
				parent.n_found_tags_ = result->n_found_tags;
			};
			c->sendGoal(goal, doneCB, OEA::SimpleActiveCallback(), OEA::SimpleFeedbackCallback());
		}

		void operator()(OKAPtr c) const
		{
			sciroc_objdet::ObjectClassificationGoal goal;
			
			auto doneCB =
			[&](const actionlib::SimpleClientGoalState &state, const sciroc_objdet::ObjectClassificationResultConstPtr &result)
			{
				parent.state_ = state;
				parent.found_tags_ = result->found_tags;
				parent.n_found_tags_ = sizeof(result->found_tags);
			};
			c->sendGoal(goal, doneCB, OKA::SimpleActiveCallback(), OKA::SimpleFeedbackCallback());
		}

		void operator()(OCAPtr c) const
		{
			sciroc_objdet::ObjectComparisonGoal goal;
			goal.expected_tags = parent.expected_tags_;

			auto doneCB =
			[&](const actionlib::SimpleClientGoalState &state, const sciroc_objdet::ObjectComparisonResultConstPtr &result)
			{
				parent.state_ = state;
				parent.found_tags_ = result->found_tags;
				parent.n_found_tags_ = sizeof(result->found_tags);

				std::sort(parent.found_tags_.begin(), parent.found_tags_.end());
				std::sort(parent.expected_tags_.begin(), parent.expected_tags_.end());
				parent.match_ = std::includes(parent.found_tags_.begin(), parent.found_tags_.end(),
																			parent.expected_tags_.begin(), parent.expected_tags_.end());
				parent.expected_tags_.clear();
			};
			c->sendGoal(goal, doneCB, OCA::SimpleActiveCallback(), OCA::SimpleFeedbackCallback());
		}
	private:
		ActionClientModeWrapper& parent;
};

class ActionClientModeWrapper::cancel_goal_visitor : public boost::static_visitor<void>
{
  public:
    void operator()(NNNPtr c) const {  return;  }
    void operator()(OEAPtr c) const {  c->cancelGoal(); }
    void operator()(OKAPtr c) const {  c->cancelGoal(); }
    void operator()(OCAPtr c) const {  c->cancelGoal(); }
};
/* TEST
*/

ActionClientModeWrapper::ActionClientModeWrapper(int mode)
: ObjDetMode(mode),
	enum_ac_(std::make_shared<OEA>("object_enumeration")),
  clas_ac_(std::make_shared<OKA>("object_classification")),
  comp_ac_(std::make_shared<OCA>("object_comparison")),
	state_(actionlib::SimpleClientGoalState::StateEnum::LOST),
	match_(false), n_found_tags_(0)
{
	ac_.insert(std::pair<Mode, OXAPtr>(Mode::ENUMERATE, enum_ac_));
  ac_.insert(std::pair<Mode, OXAPtr>(Mode::NONE, nullptr));
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
	found_tags_.clear(); //cleanup
	match_ = false;
	n_found_tags_ = 0;

	ActionClientModeWrapper::send_goal_visitor sgv = ActionClientModeWrapper::send_goal_visitor(*this);

	boost::apply_visitor(sgv, ac_[mode_]);
}
void ActionClientModeWrapper::cancelGoal()
{
	found_tags_.clear(); //cleanup
	match_ = false;
	n_found_tags_ = 0;
	/*	TODO: should we also clear the expected tags?*/

	ActionClientModeWrapper::cancel_goal_visitor cgv = ActionClientModeWrapper::cancel_goal_visitor();

	boost::apply_visitor(cgv, ac_[mode_]);

}

actionlib::SimpleClientGoalState ActionClientModeWrapper::getState() { return state_; }
bool ActionClientModeWrapper::getMatch() 	{ return match_; }
int ActionClientModeWrapper::getNumTags() { return n_found_tags_; }
std::vector<std::string> ActionClientModeWrapper::getExpectedTags() { return expected_tags_; }
std::vector<std::string> ActionClientModeWrapper::getFoundTags() 		{ return found_tags_; }

void ActionClientModeWrapper::setExpectedTags(std::vector<std::string> expected_tags)
{
	expected_tags_ = expected_tags;
}

std::ostream& operator<<(std::ostream& os, ActionClientModeWrapper o)
{
	os << "Mode: " << static_cast<ObjDetMode>(o) << std::endl;

	os << "State: " << o.getState().toString() << std::endl;

	os << "Expected tags: ";
	for (auto tag : o.getExpectedTags())
		os << tag << ", ";
	os << std::endl;

	os << "Found tags: ";
	for (auto tag : o.getFoundTags())
		os << tag << ", ";
	os << std::endl;
	
	os << "Match: " << std::boolalpha << o.getMatch() << std::endl;

	return os;
}