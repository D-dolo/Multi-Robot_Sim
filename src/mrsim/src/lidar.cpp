#include <iostream>
#include "lidar.h"

#include <opencv2/imgproc.hpp>
#include "sensor_msgs/LaserScan.h"

#include "types.h"

using namespace std;

Lidar::Lidar(int id_, string itemType_, string frame_id_, string NameSpace_1, float fov_, std::shared_ptr<World> w_,
             const Pose &pose_, float max_range_, int num_beams_, int IDparent_)
    : WorldItem(w_, pose_), num_beams(num_beams_), ranges(num_beams, -1.0), nh("~")
{

  id = id_;
  itemType = itemType_;
  frame_id = frame_id_;
  NameSpace_ = NameSpace_1;
  fov = fov_;
  max_range = max_range_;
  IDparent = IDparent_;

  // Construct the topic string using the provided NameSpace_
  string laser_topic = "/" + NameSpace_ + "/base_scan";

  // Advertise the LaserScan topic
  laser_pub = nh.advertise<sensor_msgs::LaserScan>(laser_topic, 1);
};

Lidar::Lidar(int id_, string itemType_, string frame_id_, string NameSpace_1, float fov_,
             std::shared_ptr<WorldItem> parent_, const Pose &pose_, float max_range_, int num_beams_, int IDparent_)
    : WorldItem(parent_, pose_), num_beams(num_beams_), ranges(num_beams, -1.0), nh("~")
{

  id = id_;
  itemType = itemType_;
  frame_id = frame_id_;
  NameSpace_ = NameSpace_1;
  fov = fov_;
  max_range = max_range_;
  IDparent = IDparent_;

  // Construct the topic string using the provided NameSpace_
  string laser_topic = "/" + NameSpace_ + "/base_scan";

  // Advertise the LaserScan topic
  laser_pub = nh.advertise<sensor_msgs::LaserScan>(laser_topic, 1);
};

void Lidar::timeTick(float dt)
{
  Pose piw = poseInWorld();
  IntPoint origin = world->world2grid(piw.translation());
  if (!world->inside(origin))
    return;

  float d_alpha = fov / num_beams;
  float alpha = Eigen::Rotation2Df(piw.linear()).angle() - fov / 2;
  float int_range = max_range * world->i_res;

  for (int i = 0; i < num_beams; ++i)
  {
    IntPoint endpoint;
    ranges[i] = max_range;
    bool result = world->traverseBeam(endpoint, origin, alpha, int_range);
    if (result)
    {
      IntPoint delta = endpoint - origin;
      ranges[i] = delta.norm() * world->res;
    }
    alpha += d_alpha;
  }

  // Create LaserScan message
  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.frame_id = frame_id;
  laser_msg.header.stamp = ros::Time::now();
  laser_msg.angle_min = -fov / 2;
  laser_msg.angle_max = fov / 2;
  laser_msg.angle_increment = fov / num_beams;
  laser_msg.time_increment = 0; // Set to 0 if not applicable
  laser_msg.scan_time = 0;      // Set to 0 if not applicable
  laser_msg.range_min = 0;      // Set to the minimum range of your Lidar
  laser_msg.range_max = max_range;

  // Fill the ranges array
  laser_msg.ranges = ranges;

  // Publish the LaserScan message
  laser_pub.publish(laser_msg);
}

void Lidar::draw()
{
  Pose piw = poseInWorld();
  IntPoint origin = world->world2grid(piw.translation());

  if (!world->inside(origin))
    return;

  float d_alpha = fov / num_beams;
  float alpha = -fov / 2;
  for (int i = 0; i < num_beams; ++i)
  {
    float r = ranges[i];
    Point p_lidar(r * cos(alpha), r * sin(alpha));
    Point p_world = piw * p_lidar;
    IntPoint epi = world->world2grid(p_world);
    cv::line(world->display_image, cv::Point(origin.y(), origin.x()),
             cv::Point(epi.y(), epi.x()), cv::Scalar(127, 127, 127), 1);
    alpha += d_alpha;
  }
};
