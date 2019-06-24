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
    geometry_msgs::TransformStamped baseLinkTransform;
    try {
        mouseTransform = tfBuffer.lookupTransform("base_link", trackedPoint.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    try {
        baseLinkTransform = tfBuffer.lookupTransform(trackedPoint.child_frame_id, "base_link", ros::Time(0));
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

    // base_link -> mouse
    tf2::Vector3 P;
    P.setX(mouseTransform.transform.translation.x);
    P.setY(mouseTransform.transform.translation.y);

    // mouse -> tracked point
    tf2::Vector3 X;
    X.setX(trackedPoint.twist.twist.linear.x);
    X.setY(trackedPoint.twist.twist.linear.y);

    // how the ground moved
    tf2::Transform ground_tf;
    tf2::Vector3 T;
    double aT;

    aT = DIRECTED_ANGLE(P, (P + X));
    T = P + X - P.rotate(tf2::Vector3(0.f, 0.f, 1.f), aT);
    ground_tf.setOrigin(T);
    {
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(0.f, 0.f, aT);
        ground_tf.setRotation(tf_quat);
    }

    // inverse to get base_link movement
    tf2::Transform base_link_move = ground_tf.inverse();

    // odom -> old base_link
    tf2::Transform old_base_link_tf;
    {
        tf2::Vector3 vec;
        vec.setX(baseLinkTransform.transform.translation.x);
        vec.setY(baseLinkTransform.transform.translation.y);

        tf2::Quaternion quat;

        quat.setX(baseLinkTransform.transform.rotation.x);
        quat.setY(baseLinkTransform.transform.rotation.y);
        quat.setZ(baseLinkTransform.transform.rotation.z);
        quat.setW(baseLinkTransform.transform.rotation.w);

        old_base_link_tf.setOrigin(vec);
        old_base_link_tf.setRotation(quat);
    }

    // odom -> new base_link
    tf2::Transform new_base_link_tf;
    new_base_link_tf = old_base_link_tf * base_link_move;

    ROS_DEBUG(
            "Px=%g Py=%g\n"
            "Xx=%g Xy=%g\n"
            "Tx=%g Ty=%g a=%g\n",
            P.getX(), P.getY(),
            X.getX(), X.getY(),
            T.getX(), T.getY(), aT);

    // compose transform message
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = trackedPoint.child_frame_id;
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = new_base_link_tf.getOrigin().x();
    odom_trans.transform.translation.y = new_base_link_tf.getOrigin().y();

    odom_trans.transform.rotation.x = new_base_link_tf.getRotation().x();
    odom_trans.transform.rotation.y = new_base_link_tf.getRotation().y();
    odom_trans.transform.rotation.z = new_base_link_tf.getRotation().z();
    odom_trans.transform.rotation.w = new_base_link_tf.getRotation().w();

    transformBroadcaster_ptr->sendTransform(odom_trans);
}