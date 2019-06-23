#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#define DIRECTED_ANGLE(vector1, vector2) atan2(vector2.getY(), vector2.getX()) - atan2(vector1.getY(), vector1.getX())


void odomCallback(const nav_msgs::Odometry odom);

tf2_ros::TransformBroadcaster *transformBroadcaster_ptr;
tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_combiner_node");
    ros::NodeHandle nh;

    ros::Subscriber odomSub = nh.subscribe("mouse_odom", 256, odomCallback);

    tf2_ros::TransformBroadcaster transformBroadcaster;
    transformBroadcaster_ptr = &transformBroadcaster;

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::spin();

    return 0;
}

void odomCallback(const nav_msgs::Odometry odom)
{
    geometry_msgs::TransformStamped mouseTransform;
    try {
        mouseTransform = tfBuffer.lookupTransform("base_link", odom.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
        // no movement on exception
        /*odom.pose.pose.position.x = 0.f;
        odom.pose.pose.position.y = 0.f;
        odom.pose.pose.position.z = 0.f;*/
    }

    double alpha = 0.f; // base_link rotation after translation
    tf2::Vector3 P;
    tf2::Vector3 X;

    // mouse -> odom translation
    X.setX(odom.pose.pose.position.x);
    X.setY(odom.pose.pose.position.y);
    ROS_DEBUG("Xx = %g Xy = %g", X.getX(), X.getY());

    // mouse position
    P.setX(mouseTransform.transform.translation.x);
    P.setY(mouseTransform.transform.translation.y);
    ROS_DEBUG("Px = %g Py = %g", P.getX(), P.getY());

    // alpha = tf2::tf2Angle(P, P + X);
    alpha = DIRECTED_ANGLE(P, (P + X));
    ROS_DEBUG("a = %g", alpha);

    // transform base_link -> odom
    tf2::Transform tf_transform;

    // rotation component
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0.f, 0.f, alpha);

    tf_transform.setRotation(tf_quat);

    // translation component
    tf2::Vector3 rotP;
    rotP.setX( cos(alpha)*P.getX() - sin(alpha)*P.getY() );
    rotP.setY( sin(alpha)*P.getX() + cos(alpha)*P.getY() );

    tf_transform.setOrigin(P + X - rotP);

    ROS_DEBUG("Tx = %g Ty = %g", tf_transform.getOrigin().getX(), tf_transform.getOrigin().getY());

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