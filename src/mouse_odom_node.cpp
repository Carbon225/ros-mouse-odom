#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include <unistd.h>
#include <fcntl.h>

struct MouseMove
{
    int x = 0;
    int y = 0;
};

MouseMove getMouseMove(int mouseToCapture);

int main(int argc, char **argv) {
    ros::init(argc, argv, "mouse_odom_node");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("mouse_odom", 256);
    tf2_ros::TransformBroadcaster transformBroadcaster;

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
        deltaPos = getMouseMove(in);
        current_time = ros::Time::now();

        ROS_DEBUG("x : %d | y : %d \n", deltaPos.x, deltaPos.y);

        double dt = (current_time - last_time).toSec();

        // convert DPI to meters
        double delta_x = (double)deltaPos.x / 20000.f;
        double delta_y = (double)deltaPos.y / 20000.f;
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

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.025;
        odom_trans.transform.rotation = odom_quat;

        transformBroadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.025;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;

        ros::spinOnce();
    }

    ROS_ERROR("Couldn't open mouse\n");

    return 0;
}

MouseMove getMouseMove(int mouseToCapture)
{
    unsigned char buffer[3];
    int n_read = -1;
    MouseMove result;

    n_read = read(mouseToCapture, buffer, 3);

    if (n_read == -1) {
        ROS_WARN("Error occured when tring to capture mouse movement\n");
    }

    bool x_negative = buffer[0] >> 4 & 1;
    bool y_negative = buffer[0] >> 5 & 1;

    result.x = buffer[1] - (x_negative ? 256 : 0);
    result.y = buffer[2] - (y_negative ? 256 : 0);

    return result;
}
