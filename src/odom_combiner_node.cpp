#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#define DIRECTED_ANGLE(vector1, vector2) atan2(vector2.getY(), vector2.getX()) - atan2(vector1.getY(), vector1.getX())


void odomCallback(const nav_msgs::Odometry trackedPoint);

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

void odomCallback(const nav_msgs::Odometry trackedPoint)
{
    geometry_msgs::TransformStamped mouseTransform;
    geometry_msgs::TransformStamped odomTransform;
    try {
        mouseTransform = tfBuffer.lookupTransform("base_link", trackedPoint.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    try {
        odomTransform = tfBuffer.lookupTransform("base_link", trackedPoint.child_frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());

        // send default odom -> base_link transform
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = trackedPoint.child_frame_id;
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.rotation.w = 1.f;

        transformBroadcaster_ptr->sendTransform(odom_trans);

        return;
    }

    double alphaT = 0.f;
    double alphaC = 0.f;
    double alphaO = 0.f;
    tf2::Vector3 P, X, O, T, A, B, C;

    // mouse -> tracked odom point
    X.setX(trackedPoint.pose.pose.position.x);
    X.setY(trackedPoint.pose.pose.position.y);
    ROS_DEBUG("Xx = %g Xy = %g", X.getX(), X.getY());

    // base_link -> mouse
    P.setX(mouseTransform.transform.translation.x);
    P.setY(mouseTransform.transform.translation.y);
    ROS_DEBUG("Px = %g Py = %g", P.getX(), P.getY());

    // base_link -> odom
    {
        tf2::Quaternion odom_quat;
        odom_quat.setX(odomTransform.transform.rotation.x);
        odom_quat.setY(odomTransform.transform.rotation.y);
        odom_quat.setZ(odomTransform.transform.rotation.z);
        odom_quat.setW(odomTransform.transform.rotation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(odom_quat).getRPY(roll, pitch, yaw);

        O.setX(odomTransform.transform.translation.x);
        O.setY(odomTransform.transform.translation.y);
        alphaO = yaw;

        ROS_DEBUG("Ox = %g Oy = %g Oa = %g", O.getX(), O.getY(), alphaO);
    }

    // rotate mouse pos to odom frame
    A.setX( cos(alphaO)*P.getX() - sin(alphaO)*P.getY() );
    A.setY( sin(alphaO)*P.getX() + cos(alphaO)*P.getY() );

    // mouse in odom frame -> tracked point
    B = X - O - A;

    // odom -> new odom
    alphaC = DIRECTED_ANGLE(A, (A + B));

    tf2::Vector3 rotA;
    rotA.setX( cos(alphaC)*A.getX() - sin(alphaC)*A.getY() );
    rotA.setY( sin(alphaC)*A.getX() + cos(alphaC)*A.getY() );

    C = A + B - rotA;

    // base_link -> new odom
    alphaT = alphaO + alphaC;
    T = O + C;

    // transform base_link -> odom
    tf2::Transform tf_transform;

    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0.f, 0.f, alphaT);

    tf_transform.setRotation(tf_quat);
    tf_transform.setOrigin(T);

    ROS_DEBUG("Tx = %g Ty = %g", tf_transform.getOrigin().getX(), tf_transform.getOrigin().getY());

    // inverse transform to get odom -> base_link
    tf_transform = tf_transform.inverse();

    // compose transform message
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = trackedPoint.child_frame_id;
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = tf_transform.getOrigin().x();
    odom_trans.transform.translation.y = tf_transform.getOrigin().y();
    // odom_trans.transform.translation.z = tf_transform.getOrigin().z();

    odom_trans.transform.rotation.x = tf_transform.getRotation().x();
    odom_trans.transform.rotation.y = tf_transform.getRotation().y();
    odom_trans.transform.rotation.z = tf_transform.getRotation().z();
    odom_trans.transform.rotation.w = tf_transform.getRotation().w();

    transformBroadcaster_ptr->sendTransform(odom_trans);
}