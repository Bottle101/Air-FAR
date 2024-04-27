/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include <quadrotor_msgs/GoalSet.h>

#include "goal_tool.h"

namespace rviz
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new StringProperty("Topic", "goal",
                                       "The topic on which to publish navigation goals.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
  // topic_property_droneID_ = new StringProperty("Topic", "goal_with_id",
  //                                              "The topic on which to publish navigation goals.",
  //                                              getPropertyContainer(), SLOT(updateTopic()), this);
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
  pub_droneID_goal_ = nh_.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 1);
  pub_ = nh_.advertise<geometry_msgs::PointStamped>("/goal_point", 5);
  pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  ROS_WARN("3D Goal Set");
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
           goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
           goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  pub_goal_.publish(goal);

  quadrotor_msgs::GoalSet goal_with_id;
  goal_with_id.drone_id = 0;
  goal_with_id.goal[0] = x;
  goal_with_id.goal[1] = y;
  goal_with_id.goal[2] = z;
  pub_droneID_goal_.publish(goal_with_id);

  geometry_msgs::PointStamped route_goal;
  route_goal.header.frame_id = "/map";
  route_goal.header.stamp = ros::Time::now();
  route_goal.point.x = x;
  route_goal.point.y = y;
  route_goal.point.z = z;

  pub_.publish(route_goal);
  usleep(10000);
  pub_.publish(route_goal);

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

}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::Goal3DTool, rviz::Tool)
