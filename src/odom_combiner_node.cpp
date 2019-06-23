#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


void odomCallback(const nav_msgs::Odometry odom);

tf2_ros::TransformBroadcaster *transformBroadcaster_ptr;

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_combiner_node");
    ros::NodeHandle nh;

    ros::Subscriber odomSub = nh.subscribe("mouse_odom", 256, odomCallback);
    tf2_ros::TransformBroadcaster transformBroadcaster;
    transformBroadcaster_ptr = &transformBroadcaster;

    ros::spin();

    return 0;
}

void odomCallback(const nav_msgs::Odometry odom)
{
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0.f, 0.f, 0.f);

    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = tf_quat.x();
    odom_quat.y = tf_quat.y();
    odom_quat.z = tf_quat.z();
    odom_quat.w = tf_quat.w();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = -odom.pose.pose.position.x;
    odom_trans.transform.translation.y = -odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.025;
    odom_trans.transform.rotation = odom_quat;

    transformBroadcaster_ptr->sendTransform(odom_trans);
}