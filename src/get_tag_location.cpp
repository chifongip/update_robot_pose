#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/TransformStamped.h"

#include <iostream>
#include <string>

using namespace std;

class getTagLocation
{
public: 
    getTagLocation(): tf2_buffer(), tf2_listener(tf2_buffer)
    {
        tag_detections_sub = nh.subscribe("tag_detections", 1, &getTagLocation::tagCallback, this);
        cout << "initialization successful." << endl;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber tag_detections_sub;

    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;
    geometry_msgs::TransformStamped tf2_map_tag_debug_g;

    int id;

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
    {
        if(!msg->detections.empty())
        {
            id = msg->detections[0].id[0];
            string tag_debug = "tag" + std::to_string(id);
            tf2_map_tag_debug_g = tf2_buffer.lookupTransform("map", tag_debug, ros::Time(0), ros::Duration(1.0));
            ROS_INFO("{id: %d, x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f}", id, tf2_map_tag_debug_g.transform.translation.x, tf2_map_tag_debug_g.transform.translation.y,\
                tf2_map_tag_debug_g.transform.translation.z, tf2_map_tag_debug_g.transform.rotation.x, tf2_map_tag_debug_g.transform.rotation.y,\
                tf2_map_tag_debug_g.transform.rotation.z, tf2_map_tag_debug_g.transform.rotation.w);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getTagLocation");
    getTagLocation getTagLocationNode;
    ROS_INFO("getting tag location.");
    
    ros::spin();

    return 0;
}
