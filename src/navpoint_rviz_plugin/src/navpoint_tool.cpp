#include "navpoint_tool.h"

namespace rviz
{
NavpointTool::NavpointTool()
{
  shortcut_key_ = 'w';

  topic_property_ = new StringProperty("Topic", "navpoint", "The topic on which to publish navigation waypiont for routing planner.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void NavpointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Navpoint");
  updateTopic();
  vehicle_z = 0;
}

void NavpointTool::updateTopic()
{
  sub_ = nh_.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, &NavpointTool::odomHandler, this);
  pub_ = nh_.advertise<geometry_msgs::PointStamped>("/goal_point", 5);
  pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
}

void NavpointTool::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void NavpointTool::onPoseSet(double x, double y, double theta)
{
  sensor_msgs::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = ros::Time::now();
  joy.header.frame_id = "navpoint_tool";
  pub_joy_.publish(joy);
  
  geometry_msgs::PointStamped route_goal;
  route_goal.header.frame_id = "/map";
  route_goal.header.stamp = ros::Time::now();
  route_goal.point.x = x;
  route_goal.point.y = y;
  route_goal.point.z = vehicle_z;

  pub_.publish(route_goal);
  usleep(10000);
  pub_.publish(route_goal);
}
}  // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::NavpointTool, rviz::Tool)
