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

    std::string mouse_frame = "base_link";
    if (!ros::param::get("~frame_id", mouse_frame))
        ROS_WARN("No frame_id for mouse");

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

    MouseMove deltaPos;

    while (ros::ok()) {
        if (!event_mode)
            deltaPos = getMouseMove(in);
        else
            deltaPos = getMouseMoveEvent(in);

        ros::Time current_time = ros::Time::now();

        ROS_DEBUG("x : %d | y : %d \n", deltaPos.x, deltaPos.y);

        // convert DPI to meters
        int dpi = 1000;
        if (!ros::param::get("~dpi", dpi))
            ROS_WARN("DPI not set");

        double delta_x = DOTS2M((double)deltaPos.x, dpi);
        double delta_y = DOTS2M((double)deltaPos.y, dpi);



        if (!ros::param::get("~frame_id", mouse_frame))
            ROS_WARN("No frame_id for mouse %s", device_name.c_str());

        tf2::Transform mouse_tf;
        try {
            geometry_msgs::TransformStamped mouseTransform;
            mouseTransform = tfBuffer.lookupTransform("base_link", mouse_frame, ros::Time(0));

            tf2::Vector3 v;
            v.setX(mouseTransform.transform.translation.x);
            v.setY(mouseTransform.transform.translation.y);
            v.setZ(mouseTransform.transform.translation.z);
            mouse_tf.setOrigin(v);

            tf2::Quaternion q;
            q.setX(mouseTransform.transform.rotation.x);
            q.setY(mouseTransform.transform.rotation.y);
            q.setZ(mouseTransform.transform.rotation.z);
            q.setW(mouseTransform.transform.rotation.w);
            mouse_tf.setRotation(q);
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            continue;
        }

        tf2::Transform base_link_tf;
        try {
            geometry_msgs::TransformStamped baseLinkTransform;
            baseLinkTransform = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));

            tf2::Vector3 v;
            v.setX(baseLinkTransform.transform.translation.x);
            v.setY(baseLinkTransform.transform.translation.y);
            v.setZ(baseLinkTransform.transform.translation.z);
            base_link_tf.setOrigin(v);

            tf2::Quaternion q;
            q.setX(baseLinkTransform.transform.rotation.x);
            q.setY(baseLinkTransform.transform.rotation.y);
            q.setZ(baseLinkTransform.transform.rotation.z);
            q.setW(baseLinkTransform.transform.rotation.w);
            base_link_tf.setRotation(q);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());

            // set default odom -> base_link transform
            tf2::Vector3 v(0.f, 0.f, 0.f);
            base_link_tf.setOrigin(v);

            tf2::Quaternion q;
            q.setW(1.f);
            base_link_tf.setRotation(q);
        }



        // mouse movement
        tf2::Vector3 X(delta_x, delta_y, 0.f);

        // base_link movement
        double aT = DIRECTED_ANGLE((mouse_tf.getOrigin()), (mouse_tf.getOrigin() + X));
        tf2::Vector3 T;

        tf2::Vector3 rotP;
        rotP.setX( cos(aT)*mouse_tf.getOrigin().x() - sin(aT)*mouse_tf.getOrigin().y() );
        rotP.setY( sin(aT)*mouse_tf.getOrigin().x() + cos(aT)*mouse_tf.getOrigin().y() );

        T = mouse_tf.getOrigin() + X - rotP;

        ROS_DEBUG("aT = %g", aT);

        base_link_tf.setOrigin(base_link_tf.getOrigin() + T);
        {
            tf2::Quaternion q;
            q.setRPY(0.f, 0.f, aT);

            base_link_tf.setRotation(base_link_tf.getRotation() * q);
        }

        // compose transform message
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = base_link_tf.getOrigin().x();
        odom_trans.transform.translation.y = base_link_tf.getOrigin().y();

        odom_trans.transform.rotation.x = base_link_tf.getRotation().x();
        odom_trans.transform.rotation.y = base_link_tf.getRotation().y();
        odom_trans.transform.rotation.z = base_link_tf.getRotation().z();
        odom_trans.transform.rotation.w = base_link_tf.getRotation().w();

        transformBroadcaster.sendTransform(odom_trans);

        ros::spinOnce();
    }

    ROS_ERROR("Couldn't open mouse\n");

    return 0;
}