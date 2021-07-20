#ifndef ACMW_H
#define ACMW_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "sciroc_objdet/ObjectEnumerationAction.h"
#include "sciroc_objdet/ObjectClassificationAction.h"
#include "sciroc_objdet/ObjectComparisonAction.h"
#include "sciroc_objdet/ObjDetMode.h"

#include <boost/variant.hpp>

class ActionClientModeWrapper : public ObjDetMode
{
  public:
		ActionClientModeWrapper(); // wait for servers

		void setExpectedTags(std::vector<std::string>);
		void sendGoal();
		actionlib::SimpleClientGoalState getState();
		std::vector<std::string> getFoundTags();
		bool getMatch();

	private:

		typedef actionlib::SimpleActionClient<ObjectEnumerationAction> OEA;
		typedef actionlib::SimpleActionClient<ObjectClassificationAction> OKA;
		typedef actionlib::SimpleActionClient<ObjectComparisonAction> OCA;
		typedef boost::variant<OEA, OKA, OCA> OXA;

		OEA enum_ac_;
		OKA clas_ac_;
		OCA comp_ac_;

		std::vector<std::string> expected_tags_, found_tags_;

		std::map<Mode,std::shared_ptr<OXA> > ac_;
};

#endif