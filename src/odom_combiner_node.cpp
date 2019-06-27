#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#define DIRECTED_ANGLE(vector1, vector2) atan2(vector2.getY(), vector2.getX()) - atan2(vector1.getY(), vector1.getX())


void odom1Callback(const nav_msgs::Odometry trackedPoint);
void odom2Callback(const nav_msgs::Odometry trackedPoint);


tf2::Vector3 point1, point2;
tf2::Vector3 last_point1, last_point2;


tf2::Vector3 base_link_v(0.f, 0.f, 0.f);
double base_link_a = 0.f;


tf2_ros::TransformBroadcaster *transformBroadcaster_ptr;
tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_combiner_node");
    ros::NodeHandle nh;

    std::string mouse1_frame, mouse2_frame;

    if (!ros::param::get("~mouse1", mouse1_frame)) {
        ROS_ERROR("No frame for mouse 1");
        return 1;
    }
    if (!ros::param::get("~mouse2", mouse2_frame)) {
        ROS_ERROR("No frame for mouse 2");
        return 1;
    }

    ros::Subscriber odom1Sub = nh.subscribe(mouse1_frame + "/odom", 256, odom1Callback);
    ros::Subscriber odom2Sub = nh.subscribe(mouse2_frame + "/odom", 256, odom2Callback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom_out", 256);

    tf2_ros::TransformBroadcaster transformBroadcaster;
    transformBroadcaster_ptr = &transformBroadcaster;

    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok()) {
        // calculate relative point movement
        tf2::Vector3 X1 = point1 - last_point1;
        last_point1 = point1;

        tf2::Vector3 X2 = point2 - last_point2;
        last_point2 = point2;

        ros::Time current_time = ros::Time::now();

        // get mouse positions
        tf2::Vector3 mouse1p, mouse2p;
        try {
            geometry_msgs::TransformStamped mouse1Transform =
                    tfBuffer.lookupTransform("base_link", mouse1_frame, ros::Time(0));

            geometry_msgs::TransformStamped mouse2Transform =
                    tfBuffer.lookupTransform("base_link", mouse2_frame, ros::Time(0));

            mouse1p.setX( mouse1Transform.transform.translation.x );
            mouse1p.setY( mouse1Transform.transform.translation.y );

            mouse2p.setX( mouse2Transform.transform.translation.x );
            mouse2p.setY( mouse2Transform.transform.translation.y );
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            continue;
        }

        // from point 1 to point 2 in base_link frame
        tf2::Vector3 X1X2 = (mouse2p + X2) - (mouse1p + X1);
        // from mouse 1 to mouse 2
        tf2::Vector3 mouse1mouse2 = mouse2p - mouse1p;

        // base_link rotation angle
        double alpha = DIRECTED_ANGLE(mouse1mouse2, X1X2);

        // point between 2 points
        tf2::Vector3 pointAverage = ((mouse1p + X1) + (mouse2p + X2)) / 2.f;

        // move base_link by pointAverage in base_link frame
        base_link_v += pointAverage.rotate(tf2::Vector3(0.f, 0.f, 1.f), base_link_a);

        base_link_a += alpha;

        // compose transform message
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = base_link_v.x();
        odom_trans.transform.translation.y = base_link_v.y();

        tf2::Quaternion q;
        q.setRPY(0.f, 0.f, base_link_a);

        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();

        transformBroadcaster_ptr->sendTransform(odom_trans);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = base_link_v.x();
        odom_msg.pose.pose.position.y = base_link_v.y();

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_pub.publish(odom_msg);

        ros::spinOnce();
    }

    return 0;
}

void odom1Callback(const nav_msgs::Odometry trackedPoint)
{
    point1.setX(trackedPoint.pose.pose.position.x);
    point1.setY(trackedPoint.pose.pose.position.y);
}

void odom2Callback(const nav_msgs::Odometry trackedPoint)
{
    point2.setX(trackedPoint.pose.pose.position.x);
    point2.setY(trackedPoint.pose.pose.position.y);
}