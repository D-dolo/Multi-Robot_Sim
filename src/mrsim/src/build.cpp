#include "build.h"
#include "robot.h"
#include "lidar.h"
#include "types.h"

#include <ros/ros.h>

using namespace std;

void placeObject(shared_ptr<World> worldSharedPointer, Json::Value root)
{
    Json::Value items = root["items"];
    map<int, shared_ptr<Robot>> robotsPointersMapping;
    map<int, shared_ptr<Lidar>> lidarsPointersMapping;

    for (const auto &item : items)
    {
        // Check the "type" field to determine if it's a robot or lidar
        std::string itemType = item["type"].asString();

        if (itemType == "robot")
        {
            // Extract robot configuration data
            int id = item["id"].asInt();
            std::string frame_id = item["frame_id"].asString();
            std::string NameSpace_ = item["namespace"].asString();
            float radius = item["radius"].asFloat();
            float max_rv = item["max_rv"].asFloat();
            float max_tv = item["max_tv"].asFloat();
            float x = item["pose"][0].asFloat();
            float y = item["pose"][1].asFloat();
            float theta = item["pose"][2].asFloat();
            int parent = item["parent"].asInt();

            // Initialize a robot pose
            Pose robotPose = Pose::Identity();
            robotPose.translate(worldSharedPointer->grid2world(IntPoint(x, y)));
            robotPose.rotate(theta);

            // Check if the robot is a parent or not
            if (parent == -1) // it is a parent
            {
                shared_ptr<Robot> robotSharedPtr(new Robot(id, itemType, frame_id, NameSpace_, radius, worldSharedPointer, robotPose, max_rv, max_tv, parent),
                                                 [](Robot *r) {
                                                 });
                robotsPointersMapping[id] = robotSharedPtr;
            }
            else // it is a child
            {
                shared_ptr<Robot> parentRobotSharedPtr = robotsPointersMapping[parent];
                shared_ptr<Robot> robotSharedPtr(new Robot(id, itemType, frame_id, NameSpace_, radius, parentRobotSharedPtr, robotPose, max_rv, max_tv, parent),
                                                 [](Robot *r) {
                                                 });
                robotsPointersMapping[id] = robotSharedPtr;
            }
        }
        else if (itemType == "lidar")
        {
            // Extract lidar configuration data
            int id = item["id"].asInt();
            std::string frame_id = item["frame_id"].asString();
            std::string NameSpace_ = item["namespace"].asString();
            float fov = item["fov"].asFloat();
            float max_range = item["max_range"].asFloat();
            int num_beams = item["num_beams"].asInt();
            float x = item["pose"][0].asFloat();
            float y = item["pose"][1].asFloat();
            float theta = item["pose"][2].asFloat();
            int parent = item["parent"].asInt();

            // Pose for the lidar
            Pose lidarPose = Pose::Identity();

            // Check if the lidar is a parent or not
            if (parent == -1) // Parent
            {
                shared_ptr<Lidar> lidarSharedPtr(new Lidar(id, itemType, frame_id, NameSpace_, fov, worldSharedPointer, lidarPose, max_range, num_beams, parent),
                                                 [](Lidar *l) {
                                                 });
                lidarsPointersMapping[id] = lidarSharedPtr;
            }
            else
            { // child
                lidarPose.translate(Point(x, y));
                lidarPose.rotate(theta);
                // CONSTRAINT: Lidars can be ONLY on top of robots
                shared_ptr<Robot> parentRobotSharedPtr = robotsPointersMapping[parent];
                shared_ptr<Lidar> lidarSharedPtr(new Lidar(id, itemType, frame_id, NameSpace_, fov, parentRobotSharedPtr, lidarPose, max_range, num_beams, parent),
                                                 [](Lidar *l) {
                                                 });
                lidarsPointersMapping[id] = lidarSharedPtr;
            }
        }
        else
        {
            // Handle unknown item types or other error cases
            ROS_ERROR("Unknown item type: %s", itemType.c_str());
        }
    }
    return;
}
