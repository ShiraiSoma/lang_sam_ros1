#include "lang_sam_person_following/lang_sam_person_following_node.hpp"
#include <cmath>

FollowPersonNode::FollowPersonNode()
: private_nh_("~")
{
  tracks_sub_ = nh_.subscribe(
    "/lang_sam/tracks", 1, &FollowPersonNode::tracksCallback, this);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void FollowPersonNode::tracksCallback(const lang_sam_msgs::TrackArray::ConstPtr &msg)
{
  int img_w;
  double kp_ang;
  private_nh_.param("image_width", img_w, 1280);
  private_nh_.param("kp_angular", kp_ang, 0.002);

  // BBoxは一つと仮定（先頭のみ使用）
  if (msg->tracks.empty()) return;
  const auto &t = msg->tracks.front();

  // バウンディングボックス中心（横方向のみ使用）
  double cx = (t.x_min + t.x_max) * 0.5;

  // 画面中心との偏差
  double dx = (cx - img_w * 0.5);

  geometry_msgs::Twist cmd;
  // 並進速度を常に一定(0.2 m/s)
  cmd.linear.x = 0.2;
  // 角速度は横方向偏差に比例
  cmd.angular.z = -kp_ang * dx;

  // 角速度クリップのみ維持
  if (std::abs(cmd.angular.z) > 1.0) cmd.angular.z = 1.0 * (cmd.angular.z > 0 ? 1 : -1);

  cmd_pub_.publish(cmd);
}
