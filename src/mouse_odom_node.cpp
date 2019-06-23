#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

#include <unistd.h>
#include <fcntl.h>
#include <mouseX_reader.h>
#include <eventX_reader.h>


#define DOTS2M(dots, dpi) dots * 0.0254 / dpi


int main(int argc, char **argv) {
    ros::init(argc, argv, "mouse_odom_node");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("mouse_odom", 256);

    bool event_mode = false;
    if (!ros::param::get("~event_mode", event_mode)) {
        event_mode = true;
    }

    double x = 0.f;
    double y = 0.f;
    double th = 0.f;

    double vx = 0.f;
    double vy = 0.f;
    double vth = 0.f;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = current_time;

    int in = -1;

    MouseMove deltaPos;

    std::string device_name;
    ros::param::get("~device", device_name);

    ROS_INFO("Opening %s", device_name.c_str());

    in = open(device_name.c_str(), O_RDONLY);

    while (ros::ok()) {
        if (!event_mode)
            deltaPos = getMouseMove(in);
        else
            deltaPos = getMouseMoveEvent(in);

        current_time = ros::Time::now();

        ROS_DEBUG("x : %d | y : %d \n", deltaPos.x, deltaPos.y);

        double dt = (current_time - last_time).toSec();

        // convert DPI to meters
        int dpi = 1000;
        if (!ros::param::get("~dpi", dpi))
            ROS_WARN("DPI not set");

        double delta_x = DOTS2M((double)deltaPos.x, dpi);
        double delta_y = DOTS2M((double)deltaPos.y, dpi);
        double delta_th = vth * dt;

        // calculate speed
        vx = delta_x / dt;
        vy = delta_y / dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        tf2::Quaternion tf_quat;
        tf_quat.setRPY(0.f, 0.f, th);

        geometry_msgs::Quaternion odom_quat;
        odom_quat.x = tf_quat.x();
        odom_quat.y = tf_quat.y();
        odom_quat.z = tf_quat.z();
        odom_quat.w = tf_quat.w();

        std::string mouse_frame = "base_link";
        if (!ros::param::get("~frame_id", mouse_frame))
            ROS_WARN("No frame_id for mouse %s", device_name.c_str());

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        // how the ground moved in relation to the mouse
        odom.header.frame_id = mouse_frame;
        odom.child_frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = -x;
        odom.pose.pose.position.y = -y;
        odom.pose.pose.position.z = 0; // ignore
        odom.pose.pose.orientation = odom_quat; // ignore

        //set the velocity
        odom.twist.twist.linear.x = -vx;
        odom.twist.twist.linear.y = -vy;
        odom.twist.twist.angular.z = vth; // ignore

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;

        ros::spinOnce();
    }

    ROS_ERROR("Couldn't open mouse\n");

    return 0;
}