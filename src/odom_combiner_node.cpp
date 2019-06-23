#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
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
    // transform base_link -> odom
    tf2::Transform tf_transform;

    // rotation component
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0.f, 0.f, 1.f);

    // translation component
    tf2::Vector3 tf_vector;
    tf_vector.setX(odom.pose.pose.position.x);
    tf_vector.setY(odom.pose.pose.position.y);
    tf_vector.setZ(-0.025);

    tf_transform.setOrigin(tf_vector);
    tf_transform.setRotation(tf_quat);

    // inverse transform to get odom -> base_link
    tf_transform = tf_transform.inverse();

    // compose transform message
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = tf_transform.getOrigin().x();
    odom_trans.transform.translation.y = tf_transform.getOrigin().y();
    odom_trans.transform.translation.z = tf_transform.getOrigin().z();

    odom_trans.transform.rotation.x = tf_transform.getRotation().x();
    odom_trans.transform.rotation.y = tf_transform.getRotation().y();
    odom_trans.transform.rotation.z = tf_transform.getRotation().z();
    odom_trans.transform.rotation.w = tf_transform.getRotation().w();

    transformBroadcaster_ptr->sendTransform(odom_trans);
}