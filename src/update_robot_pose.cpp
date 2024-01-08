#include "ros/ros.h"
#include "tf/tf.h"
#include "tf2_ros/transform_listener.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

#include <XmlRpcException.h>

#include <iostream>
#include <map>
#include <vector>
#include <cmath>
#include <string>

using namespace std;

class resetPose
{
public: 
    resetPose(): tf2_buffer(), tf2_listener(tf2_buffer)//, loop_rate(1)
    {
        nh.getParam("update_robot_pose/tag_locations", tag_locations);
        nh.getParam("update_robot_pose/max_detection_dist", max_detection_dist);
        nh.getParam("update_robot_pose/max_linear_vel_x", max_linear_vel_x);
        nh.getParam("update_robot_pose/max_angular_vel_z", max_angular_vel_z);
        nh.getParam("update_robot_pose/xy_tolerance", xy_tolerance);
        nh.getParam("update_robot_pose/yaw_tolerance", yaw_tolerance);
        // nh.getParam("update_robot_pose/update_frequency", update_frequency);
        nh.getParam("update_robot_pose/image_width", image_width);
        nh.getParam("update_robot_pose/image_height", image_height);
        nh.getParam("update_robot_pose/continuous_check", continuous_check);
        nh.getParam("update_robot_pose/check_threshold", check_threshold);

        reset_buf = 1;
        cnt_buf = 0;

        // construct a dictionary to store tag locations
        load_tag_poses(tag_locations, tag_poses);

        // use tf2 lookuptransform to get transformation of usb_cam_link w.r.t base_link
        tf2_base_link_usb_cam_link_g = tf2_buffer.lookupTransform("base_link", "usb_cam_link", ros::Time(0), ros::Duration(1.0));
        base_link_usb_cam_link_g.setOrigin(tf::Vector3(tf2_base_link_usb_cam_link_g.transform.translation.x, 
            tf2_base_link_usb_cam_link_g.transform.translation.y,
            tf2_base_link_usb_cam_link_g.transform.translation.z));
        base_link_usb_cam_link_g.setRotation(tf::Quaternion(tf2_base_link_usb_cam_link_g.transform.rotation.x, 
            tf2_base_link_usb_cam_link_g.transform.rotation.y,
            tf2_base_link_usb_cam_link_g.transform.rotation.z, 
            tf2_base_link_usb_cam_link_g.transform.rotation.w));

        // initialize subscriber and publisher
        tag_detections_sub = nh.subscribe("tag_detections", 1, &resetPose::poseCallback, this);
        odom_sub = nh.subscribe("odom", 1, &resetPose::odomCallback, this);
        amcl_pose_sub = nh.subscribe("amcl_pose", 1, &resetPose::amclPoseCallback, this);
        initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

        // loop_rate = ros::Rate(update_frequency);

        ROS_INFO("initialization successful.");
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber tag_detections_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber amcl_pose_sub;
    ros::Publisher initialpose_pub;
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;
    // ros::Rate loop_rate;
    // ros::WallTime start_time, end_time;
    // double execution_time;

    // double update_frequency;
    int continuous_check;
    double check_threshold;
    int image_width, image_height;
    double curr_linear_vel_x, curr_angular_vel_z;
    double max_detection_dist, max_linear_vel_x, max_angular_vel_z, xy_tolerance, yaw_tolerance;
    XmlRpc::XmlRpcValue tag_locations;
    std::map<int, geometry_msgs::Pose> tag_poses;

    int id;
    std::vector<apriltag_ros::AprilTagDetection> tag_detected;
    std::vector<double> xy_actual, xy_detect; 
    double roll_actual, pitch_actual, yaw_actual, roll_detect, pitch_detect, yaw_detect;
    double xy_diff, yaw_diff;
    
    int reset_buf, cnt_buf;
    double yaw_actual_buf, yaw_detect_buf;

    geometry_msgs::Pose map_base_link_actual_pose;
    geometry_msgs::TransformStamped tf2_map_base_link_actual_g;
    tf::Transform usb_cam_link_tag_g;
    tf::Transform tag_usb_cam_link_g;
    geometry_msgs::TransformStamped tf2_base_link_usb_cam_link_g;
    tf::Transform base_link_usb_cam_link_g;
    tf::Transform usb_cam_link_base_link_g;
    tf::Transform map_tag_g;
    tf::Transform map_base_link_g;
    geometry_msgs::PoseWithCovarianceStamped map_base_link_data;

    static bool compare_dist_from_tag(const apriltag_ros::AprilTagDetection& a, const apriltag_ros::AprilTagDetection& b);
    void load_tag_poses(const XmlRpc::XmlRpcValue& tag_poses_input, std::map<int, geometry_msgs::Pose>& tag_poses_output);
    bool is_tag_vaild(const std::vector<geometry_msgs::Point>& image_points, const int& image_width, const int& image_height);
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // get recent linear and angular velocity from odom 
        curr_linear_vel_x = abs(msg->twist.twist.linear.x);
        curr_angular_vel_z = abs(msg->twist.twist.angular.z);
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // get recent transformation of base_link w.r.t. map from amcl estimation
        map_base_link_actual_pose = msg->pose.pose;
    }

    void poseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
    {
        // start_time = ros::WallTime::now();

        // linear and angular velocity threshold for updating the robot's pose
        if(!msg->detections.empty() && curr_linear_vel_x <= max_linear_vel_x && curr_angular_vel_z <= max_angular_vel_z)
        {
            // get robot recent pose 
            // xy_actual = {map_base_link_actual_pose.position.x, map_base_link_actual_pose.position.y};
            // tf::Matrix3x3(tf::Quaternion(map_base_link_actual_pose.orientation.x, 
            //     map_base_link_actual_pose.orientation.y, 
            //     map_base_link_actual_pose.orientation.z,
            //     map_base_link_actual_pose.orientation.w)).getRPY(roll_actual, pitch_actual, yaw_actual);

            // ROS_INFO("odom stamp: %f, tag stamp: %f", tf2_map_base_link_actual_g.header.stamp.toSec(), msg->header.stamp.toSec());

            // use tag_detected vector to store the detected tags
            tag_detected.clear();
            for(int i = 0; i < msg->detections.size(); i++)
            {
                // check distance between tag and usb_cam_link, update flags if distance > threshold, 
                if(msg->detections[i].pose.pose.pose.position.z <= max_detection_dist)
                {
                    tag_detected.push_back(msg->detections[i]);
                }
            }
            
            // when the tag is detected 
            if(!tag_detected.empty())
            {
                // get the closest tag if several tags were detected 
                std::sort(tag_detected.begin(), tag_detected.end(), compare_dist_from_tag);

                // when the tag is vaild and is detected first time 
                if(is_tag_vaild(tag_detected[0].image_points, image_width, image_height))
                {
                    usb_cam_link_tag_g.setOrigin(tf::Vector3(tag_detected[0].pose.pose.pose.position.x, 
                        tag_detected[0].pose.pose.pose.position.y, 
                        tag_detected[0].pose.pose.pose.position.z));
                    usb_cam_link_tag_g.setRotation(tf::Quaternion(tag_detected[0].pose.pose.pose.orientation.x, 
                        tag_detected[0].pose.pose.pose.orientation.y,
                        tag_detected[0].pose.pose.pose.orientation.z, 
                        tag_detected[0].pose.pose.pose.orientation.w));

                    // get closest tag id
                    id = tag_detected[0].id[0];
                    
                    // get closest tag location from dictionary 
                    map_tag_g.setOrigin(tf::Vector3(tag_poses[id].position.x, 
                        tag_poses[id].position.y, 
                        tag_poses[id].position.z));
                    map_tag_g.setRotation(tf::Quaternion(tag_poses[id].orientation.x, 
                        tag_poses[id].orientation.y, 
                        tag_poses[id].orientation.z, 
                        tag_poses[id].orientation.w));

                    // calculate transformation of base_link w.r.t. map
                    tag_usb_cam_link_g = usb_cam_link_tag_g.inverse();                  // usb_cam_link w.r.t. tag
                    usb_cam_link_base_link_g = base_link_usb_cam_link_g.inverse();      // base_link w.r.t. usb_cam_link
                    
                    map_base_link_g = map_tag_g * tag_usb_cam_link_g * usb_cam_link_base_link_g;

                    tf2_map_base_link_actual_g = tf2_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
                    xy_actual = {tf2_map_base_link_actual_g.transform.translation.x, tf2_map_base_link_actual_g.transform.translation.y};
                    tf::Matrix3x3(tf::Quaternion(tf2_map_base_link_actual_g.transform.rotation.x, 
                        tf2_map_base_link_actual_g.transform.rotation.y, 
                        tf2_map_base_link_actual_g.transform.rotation.z,
                        tf2_map_base_link_actual_g.transform.rotation.w)).getRPY(roll_actual, pitch_actual, yaw_actual);

                    // get robot estimated pose based on tag 
                    xy_detect = {map_base_link_g.getOrigin().x(), map_base_link_g.getOrigin().y()};
                    tf::Matrix3x3(map_base_link_g.getRotation()).getRPY(roll_detect, pitch_detect, yaw_detect);

                    // get difference between recent pose and estimated pose
                    xy_diff = sqrt(pow((abs(xy_actual[0]) - abs(xy_detect[0])), 2) +  pow((abs(xy_actual[1]) - abs(xy_detect[1])), 2)); 
                    yaw_diff = abs(yaw_actual) - abs(yaw_detect);

                    // ROS_INFO("xy_diff: %f, yaw_diff: %f", xy_diff, yaw_diff);
                    ROS_INFO("xy_actual: %f, xy_detect: %f", xy_actual[0], xy_detect[0]);
                    ROS_INFO("xy_actual: %f, xy_detect: %f", xy_actual[1], xy_detect[1]);
                    ROS_INFO("yaw_actual: %f, yaw_detect: %f", yaw_actual, yaw_detect);
                    ROS_INFO("--------------------------------------------------------");

                    if(reset_buf)
                    {
                        ROS_INFO("counting pose.");
                        yaw_actual_buf = yaw_actual;
                        yaw_detect_buf = yaw_detect;
                        reset_buf = 0;
                    }
                    else
                    {
                        if((abs(yaw_actual_buf) - abs(yaw_actual)) < check_threshold && (abs(yaw_detect_buf) - abs(yaw_detect)) < check_threshold)
                        {
                            cnt_buf++;
                        }
                    }
                    
                    if(continuous_check == cnt_buf)
                    {
                        // update robot's pose when the difference between recent pose and estimated pose is greater than threshold 
                        if(xy_diff > xy_tolerance || yaw_diff > yaw_tolerance) 
                        {
                            // loop_rate.reset();
                            
                            map_base_link_data.header.stamp = ros::Time::now();
                            map_base_link_data.header.frame_id = "map";

                            map_base_link_data.pose.pose.position.x = map_base_link_g.getOrigin().x();
                            map_base_link_data.pose.pose.position.y = map_base_link_g.getOrigin().y();
                            map_base_link_data.pose.pose.position.z = map_base_link_g.getOrigin().z();
                            map_base_link_data.pose.pose.orientation.x = map_base_link_g.getRotation().x();
                            map_base_link_data.pose.pose.orientation.y = map_base_link_g.getRotation().y();
                            map_base_link_data.pose.pose.orientation.z = map_base_link_g.getRotation().z();
                            map_base_link_data.pose.pose.orientation.w = map_base_link_g.getRotation().w();
                            initialpose_pub.publish(map_base_link_data);

                            reset_buf = 1;
                            cnt_buf = 0;
                            ROS_INFO("resetting pose.");

                            // end_time = ros::WallTime::now();
                            // execution_time = (end_time - start_time).toNSec() * 1e-6;
                            // ROS_INFO_STREAM("Execution time: " << execution_time << " milliseconds");

                            // loop_rate.sleep();
                        }
                        else
                        {
                            reset_buf = 1;
                            cnt_buf = 0;       
                            ROS_INFO("cnt greater threshold.");                     
                        }
                    }
                }
                else
                {
                    reset_buf = 1;
                    cnt_buf = 0;
                    ROS_INFO("invaild tag.");
                }
            }
            else
            {
                reset_buf = 1;
                cnt_buf = 0; 
                ROS_INFO("outside max dist.");
            }
        }
        else
        {
            reset_buf = 1;
            cnt_buf = 0; 
            ROS_INFO("no tag detected.");
        }
    }
};

bool resetPose::compare_dist_from_tag(const apriltag_ros::AprilTagDetection& a, const apriltag_ros::AprilTagDetection& b)
{
    return a.pose.pose.pose.position.z < b.pose.pose.pose.position.z;
}

void resetPose::load_tag_poses(const XmlRpc::XmlRpcValue& tag_poses_input, std::map<int, geometry_msgs::Pose>& tag_poses_output)
{
    for(int i = 0; i < tag_poses_input.size(); i++)
    {
        geometry_msgs::Pose tag_pose_buf;
        tag_pose_buf.position.x = tag_poses_input[i]["x"];
        tag_pose_buf.position.y = tag_poses_input[i]["y"];
        tag_pose_buf.position.z = tag_poses_input[i]["z"];
        tag_pose_buf.orientation.x = tag_poses_input[i]["qx"];
        tag_pose_buf.orientation.y = tag_poses_input[i]["qy"];
        tag_pose_buf.orientation.z = tag_poses_input[i]["qz"];
        tag_pose_buf.orientation.w = tag_poses_input[i]["qw"];

        int tag_id_buf = tag_poses_input[i]["id"];

        tag_poses_output[tag_id_buf] = tag_pose_buf;
    }
}

bool resetPose::is_tag_vaild(const std::vector<geometry_msgs::Point>& image_points, const int& image_width, const int& image_height)
{
    std::vector<float> x_array, y_array;
    float x_min, x_max, y_min, y_max;

    for(int i=0; i<4; i++)
    {
        x_array.push_back(image_points[i].x);
        y_array.push_back(image_points[i].y);
    }
    
    x_min = *std::min_element(x_array.begin(), x_array.end());
    x_max = *std::max_element(x_array.begin(), x_array.end());
    y_min = *std::min_element(y_array.begin(), y_array.end());
    y_max = *std::max_element(y_array.begin(), y_array.end());

    if(x_min > 20.0 && y_min > 20.0 && x_max < (image_width - 20.0) && y_max < (image_height - 20.0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "posePublisher");
    resetPose resetPoseNode;
    ROS_INFO("reset pose node running.");
    
    ros::spin();

    return 0;
}
