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