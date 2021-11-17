#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iostream>
#include <vector>
#include <string>

#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include "cartographer_ros_msgs/LandmarkList.h"
#include "cartographer_ros_msgs/LandmarkEntry.h"

class LandmarkDetect
{
    public:
    LandmarkDetect(ros::NodeHandle nh_priv_);
    // ~LandmarkDetect();

    private:
    double translation_weight,rotation_weight;
    uint16_t tag_id;
    std::vector<int> tags;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    tf::StampedTransform transform;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped vision_pose;
    geometry_msgs::PoseStamped tag0_pose;

    std::string map_frame, camera_frame, tracking_frame;
    double publish_frequency;

    std::string family_;

    cartographer_ros_msgs::LandmarkList landmark_list;
    cartographer_ros_msgs::LandmarkEntry landmark_entry;

    private:
    void landmarkDetectCallBack(const apriltag_ros::AprilTagDetectionArray::ConstPtr &tag_detection);

};