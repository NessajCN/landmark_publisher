#include "landmark_publisher/landmark_publisher.h"

LandmarkDetect::LandmarkDetect(ros::NodeHandle nh_priv_) : nh_(nh_priv_)

{
    pub_ = nh_.advertise<cartographer_ros_msgs::LandmarkList>("landmark",1);
    sub_ = nh_.subscribe("tag_detections", 1, &LandmarkDetect::landmarkDetectCallBack, this);
    
    nh_priv_.param<std::string>("map_frame", map_frame, "/map");
    nh_priv_.param<std::string>("camera_frame", camera_frame, "/camera");
    nh_priv_.param<std::string>("tracking_frame", tracking_frame, "/base_link");
    nh_priv_.param<double>("publish_frequency",publish_frequency,10);
    nh_priv_.param<double>("rotation_weight",rotation_weight,1e8-1);
    nh_priv_.param<double>("translation_weight",translation_weight,1e8-1);

    listener.waitForTransform(tracking_frame, camera_frame, ros::Time(), ros::Duration(1.0));
}

void LandmarkDetect::landmarkDetectCallBack(const apriltag_ros::AprilTagDetectionArray::ConstPtr &tag_detections)
{
    std::vector<apriltag_ros::AprilTagDetection> tag_array = tag_detections->detections;
    
    // create header for landmark_list
    std_msgs::Header landmark_header;
    landmark_header.frame_id = tracking_frame;
    landmark_header.stamp = ros::Time::now();

    // initialize landmarl_list
    landmark_list.header = landmark_header;
    landmark_list.landmarks.clear();

    // create landmark_entry vector for landmark_list
    for (std::vector<apriltag_ros::AprilTagDetection>::iterator it = tag_array.begin(); it != tag_array.end(); ++it)
    {
        // create tracking pose
        geometry_msgs::PoseStamped camera_pose_stamped, tracking_pose_stamped;
        geometry_msgs::Pose camera_pose, tracking_pose;
        
        camera_pose = (*it).pose.pose.pose;
        // create a header for camera_pose
        std_msgs::Header camera_header;
        camera_header.frame_id = camera_frame;
        camera_header.stamp = ros::Time::now();

        camera_pose_stamped.header = camera_header;
        camera_pose_stamped.pose = camera_pose;
        listener.transformPose(tracking_frame, camera_pose_stamped, tracking_pose_stamped);
        tracking_pose = tracking_pose_stamped.pose;

        landmark_entry.tracking_from_landmark_transform = tracking_pose;
        std::string id((*it).id.begin(),(*it).id.end());
        landmark_entry.id = id;
        landmark_entry.rotation_weight = rotation_weight;
        landmark_entry.translation_weight = translation_weight;

        // append landmark_entry to vector landmark_list
        landmark_list.landmarks.push_back(landmark_entry);
    }

    // publish landmark_list to topic "/landmark"
    pub_.publish(landmark_list);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "landmark_publisher");
	ros::NodeHandle nh("~");
	LandmarkDetect LandmarkDetect(nh);
	ros::Rate loop_rate(LandmarkDetect.publish_frequency);  //10Hz
	while(ros::ok())
	{
		ros::spinOnce();

		loop_rate.sleep();
	}
    return EXIT_SUCCESS;
}
