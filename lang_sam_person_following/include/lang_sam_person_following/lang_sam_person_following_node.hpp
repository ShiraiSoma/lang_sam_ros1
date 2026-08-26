#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <lang_sam_msgs/TrackArray.h>

class FollowPersonNode
{
public:
  FollowPersonNode();

private:
  void tracksCallback(const lang_sam_msgs::TrackArray::ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber tracks_sub_;
  ros::Publisher cmd_pub_;
};
