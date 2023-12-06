#include <iostream>
#include "robot.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <opencv2/imgproc.hpp>

using namespace std;

// constructor number 1 from World
Robot::Robot(int id_, string itemType_, string frame_id_, string NameSpace_1, float radius_, std::shared_ptr<World> w_, const Pose &pose_, float max_rv_, float max_tv_, int IDparent_)
    : radius(radius_), WorldItem(w_, pose_), tv(0.0), rv(0.0), nh("~")
{
  id = id_;
  itemType = itemType_;
  frame_id = frame_id_;
  NameSpace_ = NameSpace_1;
  max_rv = max_rv_;
  max_tv = max_tv_;
  IDparent = IDparent_;

  // Construct the topic string using the provided NameSpace_
  string odom_topic = "/" + NameSpace_ + "/odom";
  string cmdvel_topic = "/" + NameSpace_ + "/cmdvel";

  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);

  // Subscribe to the cmdvel topic
  cmdvel_sub = nh.subscribe(cmdvel_topic, 1000, &Robot::cmdvelCallback, this);
}

// constructor number 2 from WorldItem
Robot::Robot(int id_, string itemType_, string frame_id_, string NameSpace_1, float radius_, std::shared_ptr<WorldItem> parent_, const Pose &pose_, float max_rv_, float max_tv_, int IDparent_)
    : radius(radius_), WorldItem(parent_, pose_), tv(0.0), rv(0.0), nh("~")
{
  id = id_;
  itemType = itemType_;
  frame_id = frame_id_;
  NameSpace_ = NameSpace_1;
  max_rv = max_rv_;
  max_tv = max_tv_;
  IDparent = IDparent_;

  // Construct the topic string using the provided NameSpace_
  string odom_topic = "/" + NameSpace_ + "/odom";
  string cmdvel_topic = "/" + NameSpace_ + "/cmdvel";

  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);

  // Subscribe to the cmdvel topic
  cmdvel_sub = nh.subscribe(cmdvel_topic, 1000, &Robot::cmdvelCallback, this);
}

void Robot::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Handle incoming velocity commands here
  // Example: Set the robot's linear and angular velocities
  tv = std::min(std::max(static_cast<float>(msg->linear.x), -max_tv), max_tv);
  rv = std::min(std::max(static_cast<float>(msg->angular.x), -max_rv), max_rv);
}

void Robot::draw()
{
  int int_radius = radius * world->i_res;
  IntPoint p = world->world2grid(poseInWorld().translation());
  cv::circle(world->display_image, cv::Point(p.y(), p.x()), int_radius,
             cv::Scalar::all(0), -1);
}

void Robot::timeTick(float dt)
{
  Pose motion = Pose::Identity();
  motion.translation() << tv * dt, 0;
  motion.rotate(rv * dt);

  Pose next_pose = pose_in_parent * motion;
  IntPoint ip = world->world2grid(next_pose.translation());
  int int_radius = radius * world->i_res;
  if (!world->collides(ip, int_radius))
    pose_in_parent = next_pose;

  // Create and populate the odometry message
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";     // map frame
  odom.child_frame_id = "base_link"; // Robot itself frame

  // Populate the pose information
  odom.pose.pose.position.x = pose_in_parent.translation().x();
  odom.pose.pose.position.y = pose_in_parent.translation().y();
  odom.pose.pose.orientation.x = Rotation(pose_in_parent.linear()).angle();
  // odom.pose.pose.orientation.y = pose_in_parent.rotation().y();
  // odom.pose.pose.orientation.z = pose_in_parent.rotation().z();
  // odom.pose.pose.orientation.w = pose_in_parent.rotation().w();

  // Publish the odometry message
  odom_pub.publish(odom);
}