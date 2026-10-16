#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>

#include <math.h>

#define STATIC_POSE 0

#define PI M_PI

#define TFOUTPUT 1
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    ros::Rate loop_rate(100);
    ros::Time start;
    bool trajectory_clock_started = false;

#if TFOUTPUT
    tf2_ros::TransformBroadcaster br;
#endif

    int count = 0;
    while (ros::ok()) {
        tf2::Vector3 origin(0,0,0);

        const ros::Time now = ros::Time::now();

        // Wait until /clock has supplied a valid simulation time.
        if (now.isZero()) {
            ros::spinOnce();
            ros::WallDuration(0.01).sleep();
            continue;
        }

        // Start trajectory time from this node's first valid clock.
        if (!trajectory_clock_started || now < start) {
            start = now;
            trajectory_clock_started = true;
            ROS_INFO("TRAJECTORY_EPOCH=%.3f", start.toSec());
        }

        const double t = (now - start).toSec();

        // Quantities to fill in
        geometry_msgs::TransformStamped desired_pose;
        desired_pose.header.stamp = ros::Time::now();
        desired_pose.header.frame_id = "world";
        desired_pose.child_frame_id = "av-desired";

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

#if STATIC_POSE
        // Static Pose
        tf2::Vector3 displacement(0,0,2);
        tf2::Vector3 trans = origin + displacement;

        desired_pose.transform.translation.x = trans.x();
        desired_pose.transform.translation.y = trans.y();
        desired_pose.transform.translation.z = trans.z();

        tf2::Quaternion q;
        q.setRPY(0,0,PI/4);
        count++;
        std::cout<<"Desired Orientation" << count << std::endl;

        desired_pose.transform.rotation.x = q.x();
        desired_pose.transform.rotation.y = q.y();
        desired_pose.transform.rotation.z = q.z();
        desired_pose.transform.rotation.w = q.w();
#else
        // Constant-radius circle with smooth angular acceleration.
        // Starts at (0,0) with zero horizontal velocity.
        const double holdTime = 15.0;
        const double rampTime = 20.0;
        const double radius = 1.0;
        const double angularSpeed = 0.10;

        const double elapsed = t - holdTime;

        double u = elapsed / rampTime;
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;

        const double u2 = u*u;
        const double u3 = u2*u;
        const double u4 = u3*u;
        const double u5 = u4*u;
        const double u6 = u5*u;

        const double smooth =
            10.0*u3 - 15.0*u4 + 6.0*u5;

        const double smoothDerivative =
            30.0*u2*(1.0-u)*(1.0-u);

        double theta;
        double thetaDot;
        double thetaDDot;

        if (elapsed <= rampTime) {
            // Integral of the quintic smoothstep.
            theta = angularSpeed * rampTime *
                (2.5*u4 - 3.0*u5 + u6);

            thetaDot = angularSpeed * smooth;
            thetaDDot =
                angularSpeed * smoothDerivative / rampTime;
        } else {
            theta = angularSpeed *
                (elapsed - rampTime/2.0);

            thetaDot = angularSpeed;
            thetaDDot = 0.0;
        }

        const double st = sin(theta);
        const double ct = cos(theta);

        desired_pose.transform.translation.x = radius*st;
        desired_pose.transform.translation.y =
            radius*(ct - 1.0);
        desired_pose.transform.translation.z = 2.0;

        velocity.linear.x = radius*ct*thetaDot;
        velocity.linear.y = -radius*st*thetaDot;
        velocity.linear.z = 0.0;

        acceleration.linear.x = radius*(
            -st*thetaDot*thetaDot
            + ct*thetaDDot
        );
        acceleration.linear.y = radius*(
            -ct*thetaDot*thetaDot
            - st*thetaDDot
        );
        acceleration.linear.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, PI/4.0);

        desired_pose.transform.rotation.x = q.x();
        desired_pose.transform.rotation.y = q.y();
        desired_pose.transform.rotation.z = q.z();
        desired_pose.transform.rotation.w = q.w();

#endif

        // Publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.transforms[0] = desired_pose.transform;
        msg.velocities.resize(1);
        msg.velocities[0] = velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = acceleration;
        desired_state_pub.publish(msg);

        std::stringstream ss;
        ss << "Trajectory Position"
           << " x:" << desired_pose.transform.translation.x
           << " y:" << desired_pose.transform.translation.y
           << " z:" << desired_pose.transform.translation.z;
        ROS_INFO_THROTTLE(2.0, "%s", ss.str().c_str());

#if TFOUTPUT
        br.sendTransform(desired_pose);
#endif

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
