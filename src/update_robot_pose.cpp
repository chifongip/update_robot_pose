#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
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
    static bool compare_dist_from_tag(const apriltag_ros::AprilTagDetection& a, const apriltag_ros::AprilTagDetection& b);
    static void load_tag_poses(const XmlRpc::XmlRpcValue& tag_poses_input, std::map<int, geometry_msgs::Pose>& tag_poses_output);

    resetPose(): tf2_buffer(), tf2_listener(tf2_buffer)
    {
        nh.getParam("update_robot_pose/tag_locations", tag_locations);
        nh.getParam("update_robot_pose/max_detection_dist", max_detection_dist);
        nh.getParam("update_robot_pose/max_linear_vel_x", max_linear_vel_x);
        nh.getParam("update_robot_pose/max_angular_vel_z", max_angular_vel_z);
        nh.getParam("update_robot_pose/xy_tolerance", xy_tolerance);
        nh.getParam("update_robot_pose/yaw_tolerance", yaw_tolerance);
        nh.getParam("update_robot_pose/update_frequency", update_frequency);
        nh.getParam("update_robot_pose/debug", debug);

        load_tag_poses(tag_locations, tag_poses);

        // wait_duration = 1.0;

        ros::Duration(1.0).sleep();

        // get transformation matrix of usb_cam_link w.r.t base_link from tf_tree
        // tf_listener.waitForTransform("base_link", "usb_cam_link", ros::Time(0), ros::Duration(wait_duration));
        // tf_listener.lookupTransform("base_link", "usb_cam_link", ros::Time(0), base_link_usb_cam_link_g);

        tf2_base_link_usb_cam_link_g = tf2_buffer.lookupTransform("base_link", "usb_cam_link", ros::Time(0));
        base_link_usb_cam_link_g.setOrigin(tf::Vector3(tf2_base_link_usb_cam_link_g.transform.translation.x, tf2_base_link_usb_cam_link_g.transform.translation.y,\
         tf2_base_link_usb_cam_link_g.transform.translation.z));
        base_link_usb_cam_link_g.setRotation(tf::Quaternion(tf2_base_link_usb_cam_link_g.transform.rotation.x, tf2_base_link_usb_cam_link_g.transform.rotation.y, \
         tf2_base_link_usb_cam_link_g.transform.rotation.z, tf2_base_link_usb_cam_link_g.transform.rotation.w));

        tag_detections_sub = nh.subscribe("tag_detections", 1, &resetPose::poseCallback, this);
        odom_sub = nh.subscribe("odom", 1, &resetPose::odomCallback, this);
        initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

        cout << "success initialization." << endl;

        // for(const auto& elem: tag_poses)
        // {
        //     cout << elem.first << ": " << elem.second << endl;
        // }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        curr_linear_vel_x = abs(msg->twist.twist.linear.x);
        curr_angular_vel_z = abs(msg->twist.twist.angular.z);
    }

    void poseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
    {
        // start_time = ros::WallTime::now();
        // linear and angular velocity threshold for updating the robot's pose

        tf2_map_base_link_actual_g = tf2_buffer.lookupTransform("map", "base_link", ros::Time(0));

        // cout << "****************************************************************************" << endl;
        // cout << "lookuptransform stamp: " << endl << tf2_map_base_link_actual_g.header.stamp << endl;
        // cout << "----------------------------------------------------------------------------" << endl;
        // cout << "tag detection stamp: " << endl << msg->header.stamp << endl;
        // cout << "****************************************************************************" << endl;

        if(!msg->detections.empty() && curr_linear_vel_x <= max_linear_vel_x && curr_angular_vel_z <= max_angular_vel_z)
        {
            // ros::Rate loop_rate(update_frequency);

            tag_detected.clear();
            for(int i = 0; i < msg->detections.size(); i++)
            {
                // distance between tag and base_link
                if(msg->detections[i].pose.pose.pose.position.z <= max_detection_dist)
                {
                    tag_detected.push_back(msg->detections[i]);
                }
            }
            
            if(!tag_detected.empty())
            {
                std::sort(tag_detected.begin(), tag_detected.end(), compare_dist_from_tag);

                usb_cam_link_tag_g.setOrigin(tf::Vector3(tag_detected[0].pose.pose.pose.position.x, tag_detected[0].pose.pose.pose.position.y, tag_detected[0].pose.pose.pose.position.z));
                usb_cam_link_tag_g.setRotation(tf::Quaternion(tag_detected[0].pose.pose.pose.orientation.x, tag_detected[0].pose.pose.pose.orientation.y,\
                 tag_detected[0].pose.pose.pose.orientation.z, tag_detected[0].pose.pose.pose.orientation.w));

                // int id = 0;
                // for(auto d: tag_detected[0].id)
                // {
                //     id = id * 10 + d;
                // }

                // for(int i = 0; i < tag_locations.size(); i++)
                // {
                //     if((int) tag_locations[i]["id"] == id)
                //     {
                //         map_tag_g.setOrigin(tf::Vector3(tag_locations[i]["x"], tag_locations[i]["y"], tag_locations[i]["z"]));
                //         map_tag_g.setRotation(tf::Quaternion(tag_locations[i]["qx"], tag_locations[i]["qy"], tag_locations[i]["qz"], tag_locations[i]["qw"]));
                //     }
                // }

                int id = tag_detected[0].id[0];

                map_tag_g.setOrigin(tf::Vector3(tag_poses[id].position.x, tag_poses[id].position.y, tag_poses[id].position.z));
                map_tag_g.setRotation(tf::Quaternion(tag_poses[id].orientation.x, tag_poses[id].orientation.y, tag_poses[id].orientation.z, tag_poses[id].orientation.w));

                // calculation of base_link w.r.t. map
                tag_usb_cam_link_g = usb_cam_link_tag_g.inverse();                  // usb_cam_link w.r.t. tag
                usb_cam_link_base_link_g = base_link_usb_cam_link_g.inverse();      // base_link w.r.t. usb_cam_link
                
                map_base_link_g = map_tag_g * tag_usb_cam_link_g * usb_cam_link_base_link_g;
                
                // tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(wait_duration));
                // tf_listener.lookupTransform("map", "base_link", ros::Time(0), map_base_link_actual_g);
                
                xy_actual = {tf2_map_base_link_actual_g.transform.translation.x, tf2_map_base_link_actual_g.transform.translation.y};
                tf::Matrix3x3(tf::Quaternion(tf2_map_base_link_actual_g.transform.rotation.x, tf2_map_base_link_actual_g.transform.rotation.y, tf2_map_base_link_actual_g.transform.rotation.z,\
                 tf2_map_base_link_actual_g.transform.rotation.w)).getRPY(roll_actual, pitch_actual, yaw_actual);
                
                // xy_actual = {map_base_link_actual_g.getOrigin().x(), map_base_link_actual_g.getOrigin().y()};       // actual xy position
                // // map_base_link_actual_q = map_base_link_actual_g.getRotation();
                // tf::Matrix3x3(map_base_link_actual_g.getRotation()).getRPY(roll_actual, pitch_actual, yaw_actual);  // actual yaw angle
                
                // get rotation and translation matrix from transformation matrix 
                xy_detect = {map_base_link_g.getOrigin().x(), map_base_link_g.getOrigin().y()};
                // map_base_link_q = map_base_link_g.getRotation();
                tf::Matrix3x3(map_base_link_g.getRotation()).getRPY(roll_detect, pitch_detect, yaw_detect);

                xy_diff = sqrt(pow((xy_actual[0] - xy_detect[0]), 2) +  pow((xy_actual[1] - xy_detect[1]), 2)); 
                yaw_diff = abs(yaw_actual - yaw_detect);

                // cout << "xy_diff: " << xy_diff << "------" << "yaw_diff: " << yaw_diff << endl;
                // cout << "------------------------------------------------------------" << endl;

                if(debug == 1)
                {
                    string tag_debug = "tag" + std::to_string(id);
                    // tf_listener.waitForTransform("map", tag_debug, ros::Time(0), ros::Duration(wait_duration));
                    // tf_listener.lookupTransform("map", tag_debug, ros::Time(0), map_tag_debug_g);
                    tf2_map_tag_debug_g = tf2_buffer.lookupTransform("map", tag_debug, ros::Time(0));

                    // ROS_INFO("{id: %d, x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f}", id, map_tag_debug_g.getOrigin().x(), map_tag_debug_g.getOrigin().y(),\
                    //  map_tag_debug_g.getOrigin().z(), map_tag_debug_g.getRotation().x(), map_tag_debug_g.getRotation().y(), map_tag_debug_g.getRotation().z(),\
                    //  map_tag_debug_g.getRotation().w());

                    ROS_INFO("{id: %d, x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f}", id, tf2_map_tag_debug_g.transform.translation.x, tf2_map_tag_debug_g.transform.translation.y,\
                     tf2_map_tag_debug_g.transform.translation.z, tf2_map_tag_debug_g.transform.rotation.x, tf2_map_tag_debug_g.transform.rotation.y,\
                     tf2_map_tag_debug_g.transform.rotation.z, tf2_map_tag_debug_g.transform.rotation.w);
                }
                else
                {
                    // xy and yaw tolerance for updating the robot's pose
                    if(xy_diff > xy_tolerance || yaw_diff > yaw_tolerance)
                    {
                        // map_base_link_data.header.seq = msg->header.seq;
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

                        // end_time = ros::WallTime::now();
                        // execution_time = (end_time - start_time).toNSec() * 1e-6;
                        // ROS_INFO_STREAM("Execution time: " << execution_time << " milliseconds");

                        // loop_rate.sleep();
                    }
                }
            }
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber tag_detections_sub;
    ros::Subscriber odom_sub;
    ros::Publisher initialpose_pub;

    // double wait_duration; 
    // ros::WallTime start_time, end_time;
    // double execution_time;

    int debug;
    double update_frequency;
    double curr_linear_vel_x, curr_angular_vel_z;
    double max_detection_dist, max_linear_vel_x, max_angular_vel_z, xy_tolerance, yaw_tolerance;
    XmlRpc::XmlRpcValue tag_locations;
    std::map<int, geometry_msgs::Pose> tag_poses;

    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;
    geometry_msgs::TransformStamped tf2_map_base_link_actual_g;
    geometry_msgs::TransformStamped tf2_base_link_usb_cam_link_g;
    geometry_msgs::TransformStamped tf2_map_tag_debug_g;
    tf::Transform base_link_usb_cam_link_g;

    // tf::Transformer tf_tool;
    // tf::TransformListener tf_listener;
    // tf::StampedTransform base_link_usb_cam_link_g;
    // tf::StampedTransform map_base_link_actual_g;
    // tf::Quaternion map_base_link_actual_q;
    // tf::Quaternion map_base_link_q;

    geometry_msgs::PoseWithCovarianceStamped map_base_link_data;
    std::vector<apriltag_ros::AprilTagDetection> tag_detected;

    std::vector<double> xy_actual, xy_detect; 
    double roll_actual, pitch_actual, yaw_actual, roll_detect, pitch_detect, yaw_detect;
    double xy_diff, yaw_diff;

    tf::Transform usb_cam_link_tag_g;
    tf::Transform map_tag_g;

    tf::Transform tag_usb_cam_link_g;
    tf::Transform usb_cam_link_base_link_g;
    tf::Transform map_base_link_g; 

    // tf::StampedTransform map_tag_debug_g;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "posePublisher");
    resetPose resetPoseNode;
    ROS_INFO("reset pose node running.");
    
    ros::spin();

    return 0;
}
