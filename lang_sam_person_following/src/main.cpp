#include <ros/ros.h>
#include "lang_sam_person_following/lang_sam_person_following_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lang_sam_person_following_node");
  FollowPersonNode node;
  ros::spin();
  return 0;
}
