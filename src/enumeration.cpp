#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "sciroc_objdet/ObjectEnumerationAction.h"

#include <stdlib.h>
#include <time.h>

class ObjectEnumerationAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sciroc_objdet::ObjectEnumerationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  sciroc_objdet::ObjectEnumerationResult result_;

public:

  ObjectEnumerationAction(std::string name) :
    as_(nh_, name, boost::bind(&ObjectEnumerationAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
		srand(time(NULL));
	}

	~ObjectEnumerationAction(void)
  {
  }

  void executeCB(const sciroc_objdet::ObjectEnumerationGoalConstPtr &goal)
  {
    // helper variables
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("%s: Action started", action_name_.c_str());

    // start executing the action
		ros::Duration(2 + rand() % 10).sleep();

		if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
			result_.n_found_tags = 3;
			as_.setSucceeded(result_);
		}
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  ObjectEnumerationAction enum_as("object_enumeration");
  ros::spin();

  return 0;
}