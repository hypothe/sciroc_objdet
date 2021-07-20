#include "sciroc_objdet/ActionClientModeWrapper.h"

class ActionClientModeWrapper::wait_visitor : public boost::static_visitor<bool>
{
  public:
    bool operator()(std::shared_ptr<NNN> c){  return true;  }
    bool operator()(std::shared_ptr<OEA> c){  return c->waitForServer(wait_time); }
    bool operator()(std::shared_ptr<OKA> c){  return c->waitForServer(wait_time); }
    bool operator()(std::shared_ptr<OCA> c){  return c->waitForServer(wait_time); }

  private:
    ros::Duration wait_time = ros::Duration(1.0);
};

class ActionClientModeWrapper::send_goal_visitor : public boost::static_visitor<>
{
	public:
		send_goal_visitor(ActionClientModeWrapper &p) : parent(p){}

		void operator()(std::shared_ptr<NNN> c) { return; }

		void operator()(std::shared_ptr<OEA> c)
		{
			sciroc_objdet::ObjectEnumerationGoal goal;
			auto doneCB = [this](const actionlib::SimpleClientGoalState &state, const sciroc_objdet::ObjectEnumerationResultConstPtr &result)
			{
				parent.state_ = state.state_;
				parent.n_found_tags = result->n_found_tags;
			};
			c->sendGoal(goal, boost::bind(doneCB, this, std::placeholders::_1, std::placeholders::_2), OEA::SimpleActiveCallback(), OEA::SimpleFeedbackCallback());
		}

		void operator()(std::shared_ptr<OKA> c)
		{
			sciroc_objdet::ObjectClassificationGoal goal;
			auto doneCB = [this](const actionlib::SimpleClientGoalState &state, const sciroc_objdet::ObjectClassificationResultConstPtr &result)
			{
				parent.state_ = state.state_;
				parent.found_tags_ = result->found_tags;
			};
			c->sendGoal(goal, boost::bind(doneCB, this, std::placeholders::_1, std::placeholders::_2), OKA::SimpleActiveCallback(), OKA::SimpleFeedbackCallback());
		}

		void operator()(std::shared_ptr<OCA> c)
		{
			sciroc_objdet::ObjectComparisonGoal goal;
			auto doneCB = [this](const actionlib::SimpleClientGoalState &state, const sciroc_objdet::ObjectComparisonResultConstPtr &result)
			{
				parent.state_ = state.state_;
				parent.found_tags_ = result->found_tags;
				std::sort(parent.found_tags_.begin(), parent.found_tags_.end());
				std::sort(parent.expected_tags_.begin(), parent.expected_tags_.end());
				parent.match_ = std::includes(parent.found_tags_.begin(), parent.found_tags_.end(),
																			parent.expected_tags_.begin(), parent.expected_tags_.end());
				parent.expected_tags_.clear();
			};
			c->sendGoal(goal, boost::bind(doneCB, this, std::placeholders::_1, std::placeholders::_2), OCA::SimpleActiveCallback(), OCA::SimpleFeedbackCallback());
		}

	private:
		ActionClientModeWrapper& parent;
};

ActionClientModeWrapper::ActionClientModeWrapper()
: enum_ac_("object_enumeration"),
  clas_ac_("object_classification"),
  comp_ac_("object_comparison")
{
  ac_.insert(std::pair<Mode, std::shared_ptr<OXA> >(Mode::NONE, nullptr));
  ac_.insert(std::pair<Mode, std::shared_ptr<OXA> >(Mode::ENUMERATE, std::make_shared<OXA>(enum_ac_)));
  ac_.insert(std::pair<Mode, std::shared_ptr<OXA> >(Mode::CLASSIFY, std::make_shared<OXA>(clas_ac_)));
  ac_.insert(std::pair<Mode, std::shared_ptr<OXA> >(Mode::COMPARE, std::make_shared<OXA>(comp_ac_)));
}

void ActionClientModeWrapper::waitForServer()
{
  bool servers_up = false;

  while(!servers_up)
  {
    servers_up = true;
    for (auto mode_ac : ac_)
    {
      servers_up = servers_up && boost::apply_visitor(ActionClientModeWrapper::wait_visitor(), mode_ac.second);
    }
  }
}

void ActionClientModeWrapper::sendGoal()
{
	found_tags_.clear(); //cleanup
	match_ = false;
	n_found_tags = 0;

	ActionClientModeWrapper::send_goal_visitor sgv = ActionClientModeWrapper::send_goal_visitor(*this);

	boost::apply_visitor(sgv, ac_[mode_]);
}

actionlib::SimpleClientGoalState::StateEnum ActionClientModeWrapper::getState() { return state_; }
bool ActionClientModeWrapper::getMatch() 	{ return match_; }
int ActionClientModeWrapper::getNumTags() { return n_found_tags; }