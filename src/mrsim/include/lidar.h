#pragma once
#include <vector>

#include "world.h"
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

class Lidar : public WorldItem
{
public:
      Lidar(int id_, string itemType_, string frame_id_, string NameSpace_1, float fov_, std::shared_ptr<World> w_,
            const Pose &pose_ = Pose::Identity(), float max_range_ = 10.0, int num_beams_ = 180, int IDparent = -1);

      Lidar(int id_, string itemType_, string frame_id_, string NameSpace_1, float fov_, std::shared_ptr<WorldItem> p_,
            const Pose &pose_ = Pose::Identity(), float max_range_ = 10.0, int num_beams_ = 180, int IDparent = -1);

      void timeTick(float dt) override;

      void draw() override;

      int id;
      string itemType;
      string frame_id;
      string NameSpace_;
      float fov;
      float max_range;
      int num_beams;
      int IDparent;
      std::vector<float> ranges;

      ros::NodeHandle nh;
      ros::Publisher laser_pub;
};