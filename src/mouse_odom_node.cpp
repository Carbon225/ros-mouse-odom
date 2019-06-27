#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <unistd.h>
#include <fcntl.h>
#include <mouseX_reader.h>
#include <eventX_reader.h>


#define DIRECTED_ANGLE(vector1, vector2) atan2(vector2.getY(), vector2.getX()) - atan2(vector1.getY(), vector1.getX())
#define DOTS2M(dots, dpi) dots * 0.0254 / dpi


int main(int argc, char **argv) {
    ros::init(argc, argv, "mouse_odom_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster transformBroadcaster;

    std::string mouse_frame;
    if (!ros::param::get("~frame_id", mouse_frame)) {
        ROS_ERROR("No frame_id for mouse");
        return 1;
    }

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(mouse_frame + "/odom", 256);

    bool event_mode = false;
    if (!ros::param::get("~event_mode", event_mode)) {
        event_mode = true;
    }

    std::string device_name;
    if (!ros::param::get("~device", device_name)) {
        ROS_ERROR("No device set for mouse");
        return 1;
    }

    ROS_INFO("Opening %s", device_name.c_str());

    int in = -1;
    in = open(device_name.c_str(), O_RDONLY);

    tf2::Vector3 mouse_point;

    MouseMove deltaPos;

    while (ros::ok()) {
        if (!event_mode)
            deltaPos = getMouseMove(in);
        else
            deltaPos = getMouseMoveEvent(in);

        ros::Time current_time = ros::Time::now();

        // ROS_DEBUG("x : %d | y : %d \n", deltaPos.x, deltaPos.y);

        // convert DPI to meters
        int dpi = 1000;
        if (!ros::param::get("~dpi", dpi))
            ROS_WARN("DPI not set");

        tf2::Vector3 X;

        X.setX( DOTS2M((double)deltaPos.x, dpi) );
        X.setY( DOTS2M((double)deltaPos.y, dpi) );

        mouse_point += X;

        // compose transform message
        {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = current_time;
            tf_msg.header.frame_id = mouse_frame;
            tf_msg.child_frame_id = mouse_frame + "/odom";

            tf_msg.transform.translation.x = mouse_point.x();
            tf_msg.transform.translation.y = mouse_point.y();

            tf_msg.transform.rotation.w = 1.f;

            transformBroadcaster.sendTransform(tf_msg);

            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = mouse_frame;
            odom_msg.child_frame_id = mouse_frame + "/odom";

            odom_msg.pose.pose.position.x = mouse_point.x();
            odom_msg.pose.pose.position.y = mouse_point.y();

            odom_msg.pose.pose.orientation.w = 1.f;

            odom_pub.publish(odom_msg);
        }

        ros::spinOnce();
    }

    ROS_ERROR("Couldn't open mouse\n");

    return 0;
}