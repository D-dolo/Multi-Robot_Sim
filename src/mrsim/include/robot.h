#pragma once

#include "types.h"
#include "world.h"
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

using namespace std;

struct Robot : public WorldItem
{
      Robot(int id_, string itemType_, string frame_id_, string NameSpace_1, float radius_, std::shared_ptr<World> w_, const Pose &pose_ = Pose::Identity(), float max_rv_ = 0.0, float max_tv_ = 0.0, int IDparent = -1);

      Robot(int id_, string itemType_, string frame_id_, string NameSpace_1, float radius_, std::shared_ptr<WorldItem> parent_, const Pose &pose_ = Pose::Identity(), float max_rv_ = 0.0, float max_tv_ = 0.0, int IDparent = -1);

      void draw() override;
      void timeTick(float dt) override;

      void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);

      int id;
      string itemType;
      string frame_id;
      string NameSpace_;
      float radius;
      float tv = 0, rv = 0;
      float max_rv;
      float max_tv;
      int IDparent;

      ros::NodeHandle nh;
      nav_msgs::Odometry odom;
      ros::Publisher odom_pub;
      ros::Subscriber cmdvel_sub;

};