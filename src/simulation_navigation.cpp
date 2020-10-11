#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_turtlebot3_simulation/simulation_navigator.h>
#include <multi_turtlebot3_simulation/simulation_task_distributor.h>

void sendGoal(const std::string& actionlib, const geometry_msgs::PoseStamped::ConstPtr& poseStamped)
{
  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(actionlib, true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal move_base_goal;
  move_base_goal.target_pose.header.frame_id = "map";
  move_base_goal.target_pose.header.stamp    = ros::Time::now();

  move_base_goal.target_pose.pose.position.x = poseStamped->pose.position.x;
  move_base_goal.target_pose.pose.position.y = poseStamped->pose.position.y;
  move_base_goal.target_pose.pose.position.z = poseStamped->pose.position.z;
  move_base_goal.target_pose.pose.orientation.x = poseStamped->pose.orientation.x;
  move_base_goal.target_pose.pose.orientation.y = poseStamped->pose.orientation.y;
  move_base_goal.target_pose.pose.orientation.z = poseStamped->pose.orientation.z;
  move_base_goal.target_pose.pose.orientation.w = poseStamped->pose.orientation.w;

  ROS_INFO("Sending goal");
  ac.sendGoal(move_base_goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, naviagtion goal send");
  else
    ROS_INFO("Robot failed to move");
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& poseStamped)
{
  ROS_INFO("I heard a goal!!!");
  ROS_INFO("header.frame_id: [%s]", poseStamped->header.frame_id.c_str());
  ROS_INFO("header.stamp: [%f]", poseStamped->header.stamp.toSec());
  ROS_INFO("pose.position.x: [%f]", poseStamped->pose.position.x);
  ROS_INFO("pose.position.y: [%f]", poseStamped->pose.position.y);
  ROS_INFO("pose.position.z: [%f]", poseStamped->pose.position.z);

  // Get navigation distamce of turtlebot01
  double d1=0;
  while (d1 == 0) {
    ROS_INFO("Get navigation distamce of turtlebot01...");
    multi_turtlebot3_simulation::SimulationNavigator navigator("turtlebot01", "base_footprint", "map", "move_base/NavfnROS/make_plan");
    d1 = navigator.getDistance(*poseStamped);
    ROS_INFO("d1: [%f]", d1);
  }

  // Get navigation distamce of turtlebot02
  double d2=0;
  while (d2 == 0) {
    ROS_INFO("Get navigation distamce of turtlebot02..");
    multi_turtlebot3_simulation::SimulationNavigator navigator("turtlebot02", "base_footprint", "map", "move_base/NavfnROS/make_plan");
    d2 = navigator.getDistance(*poseStamped);
    ROS_INFO("d2: [%f]", d2);
  }

  // Get navigation plan of turtlebot03
  double d3=0;
  while (d3 == 0) {
    ROS_INFO("Get navigation distamce of turtlebot03..");
    multi_turtlebot3_simulation::SimulationNavigator navigator("turtlebot03", "base_footprint", "map", "move_base/NavfnROS/make_plan");
    d3 = navigator.getDistance(*poseStamped);
    ROS_INFO("d3: [%f]", d3);
  }

  
  // Send goal to shortest distance
  if (d1 <= d2 && d1 <= d3) {
    sendGoal("/turtlebot01/move_base", poseStamped);
  } else if (d2 <= d1 && d2 <= d3) {
    sendGoal("/turtlebot02/move_base", poseStamped);
  } else {
    sendGoal("/turtlebot03/move_base", poseStamped);
  }
}

std::vector<geometry_msgs::PoseStamped> goals;

void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& poseStamped)
{
  goals.insert(poseStamped);

  if (goals.size() == 3)
  {
    executeSeq();
    goals = std::vector<geometry_msgs::PoseStamped>(); // reset goals
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "simulation_navigation");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 1000, setGoalCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}