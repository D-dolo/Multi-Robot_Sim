#include <iostream>
#include <json/json.h>
#include <fstream>

#include "lidar.h"
#include "robot.h"
#include "types.h"
#include "world.h"
#include "build.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/highgui.hpp>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrsim_node");

  // Parse the JSON file
  Json::Value root; // Create a root JSON object
  Json::Reader reader;
  ifstream jsonFile("/home/giancarlo/Multi-Robot Simulator RobotProgramming/src/mrsim/data/cappero_2r.json", ifstream::binary);

  if (!reader.parse(jsonFile, root, false))
  {
    std::cerr << "Error parsing JSON file: " << reader.getFormattedErrorMessages() << std::endl;
    return 1;
  }

  // Extract map value from the JSON
  string mapName = root["map"].asString();  //deserialization of the json file

  World w;
  w.loadFromImage(("/home/giancarlo/Multi-Robot Simulator RobotProgramming/src/mrsim/data/" + mapName).c_str());

  shared_ptr<World> worldSharedPtr(&w,
                                   [](World *w)
                                   {
                                     // Custom cleanup actions here, if needed
                                     // delete w; // Clean up the Robot object
                                   });

  placeObject(worldSharedPtr, root);
  // IntPoint middle(w.rows / 2, w.cols / 2);

  // Pose robot_pose = Pose::Identity();
  // robot_pose.translate(w.grid2world(middle));
  // robot_pose.rotate(0.0);

  // // Create robot instances with initial positions
  // Robot robot1(0.6, worldSharedPtr, robot_pose); // Example initial pose

  // Create a shared pointer for robot1
  // shared_ptr<Robot> robot1SharedPtr = make_shared<Robot>(robot1);

  // // Use robot1SharedPtr to create robot2 related to robot1
  // Pose robot2_pose = Pose::Identity();
  // robot2_pose.translate(w.grid2world(middle)); // Adjust the pose as needed
  // robot2_pose.rotate(0.0);
  // Robot robot2(0.6, robot1SharedPtr, robot2_pose); // Create robot2 related to robot1

  int k;
  float delay = 0.07;

  while (ros::ok())
  {
    // Update the state of each robot
    w.draw();
    w.timeTick(delay);

    k = cv::waitKeyEx(delay * 1000) & 255;
    switch (k)
    {
    case 27:
      return 0; // space
    default:;
    }

    cv::waitKey(100);
    ros::spinOnce();
  }

  return 0;
}