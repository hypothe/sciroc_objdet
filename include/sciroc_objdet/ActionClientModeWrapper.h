#ifndef ACMW_H
#define ACMW_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include "sciroc_objdet/ObjectEnumerationAction.h"
#include "sciroc_objdet/ObjectClassificationAction.h"
#include "sciroc_objdet/ObjectComparisonAction.h"
#include "sciroc_objdet/ObjDetMode.h"

#include <boost/variant.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

/*	TODO: is it worth making this class a child of SimpleActionClient?	*/

class ActionClientModeWrapper : public ObjDetMode
{
  public:
		ActionClientModeWrapper();
		ActionClientModeWrapper(int mode);

		void waitForServer();
		void waitForAllServers();

		void setExpectedTags(std::vector<std::string>);
		void setTablePos(geometry_msgs::Point table_pos);
		void sendGoal();
		void cancelGoal();
		void doneCB(const actionlib::SimpleClientGoalState &state,
								const sciroc_objdet::ObjectEnumerationResultConstPtr &result);
		void doneCB(const actionlib::SimpleClientGoalState &state,
								const sciroc_objdet::ObjectClassificationResultConstPtr &result);
		void doneCB(const actionlib::SimpleClientGoalState &state,
								const sciroc_objdet::ObjectComparisonResultConstPtr &result);
		actionlib::SimpleClientGoalState getState();

		int getNumTags();
		std::vector<std::string> getExpectedTags();
		std::vector<std::string> getFoundTags();
		bool getMatch();

		std::string getText();

	private:

		typedef std::nullptr_t NNNPtr;
		typedef actionlib::SimpleActionClient<sciroc_objdet::ObjectEnumerationAction> OEA;
		typedef actionlib::SimpleActionClient<sciroc_objdet::ObjectClassificationAction> OKA;
		typedef actionlib::SimpleActionClient<sciroc_objdet::ObjectComparisonAction> OCA;

		typedef std::shared_ptr<OEA> OEAPtr; 
		typedef std::shared_ptr<OKA> OKAPtr;
		typedef std::shared_ptr<OCA> OCAPtr;
		typedef boost::variant<NNNPtr, OEAPtr, OKAPtr, OCAPtr> OXAPtr;

		OEAPtr enum_ac_;
		OKAPtr clas_ac_;
		OCAPtr comp_ac_;

		boost::shared_mutex mutexResult_; // used to correctly access and modify the resources
		std::vector<std::string> expected_tags_, found_tags_;
		geometry_msgs::Point table_pos_;
		bool match_;
		int n_found_tags_;
		actionlib::SimpleClientGoalState state_;

		std::map<Mode,OXAPtr> ac_;


		class wait_visitor;
		class send_goal_visitor;
		class cancel_goal_visitor;
		class get_state_visitor;

};

std::ostream &operator<<(std::ostream &os, ActionClientModeWrapper o);

#endif